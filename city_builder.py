# city_builder.py
import osmnx as ox
import geopandas as gpd
import numpy as np
from shapely.geometry import Polygon, MultiPoint, Point
from shapely.ops import voronoi_diagram
import config as cfg

def gerar_cidade():
    if not cfg.CENARIO_SEMPRE_NOVO: np.random.seed(42)

    graph = ox.graph_from_point(cfg.COORDENADAS, dist=cfg.RAIO_M, network_type='drive')
    ruas = ox.graph_to_gdfs(graph, nodes=False, edges=True).to_crs(epsg=3857)

    bbox = ruas.total_bounds
    ambiente = Polygon([(bbox[0], bbox[1]), (bbox[2], bbox[1]), (bbox[2], bbox[3]), (bbox[0], bbox[3])])
    asfalto = ruas.geometry.buffer(cfg.LARGURA_RUA / 2).union_all()
    quarteiroes = gpd.GeoDataFrame(geometry=[ambiente.difference(asfalto)]).explode(index_parts=False)

    lotes_list = []
    for q in quarteiroes.geometry:
        num = int(q.area / cfg.AREA_MEDIA_LOTE)
        pts = []
        minx, miny, maxx, maxy = q.bounds
        while len(pts) < num:
            p = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
            if q.contains(p): pts.append(p)
        if len(pts) >= 2:
            for poly in voronoi_diagram(MultiPoint(pts), envelope=q.buffer(10)).geoms:
                res = poly.intersection(q).buffer(-cfg.ESPACO_ENTRE_LOTES)
                if not res.is_empty and res.area > 30: lotes_list.append(res)

    lotes_gdf = gpd.GeoDataFrame(geometry=lotes_list, crs=ruas.crs)
    minx, miny, _, _ = lotes_gdf.total_bounds
    lotes_gdf.geometry = lotes_gdf.translate(xoff=-minx, yoff=-miny)
    max_x, max_y = int(lotes_gdf.total_bounds[2]), int(lotes_gdf.total_bounds[3])

    lotes_gdf['lote_id'] = range(len(lotes_gdf))
    ponto_inicio = Point(cfg.START_LOC)
    ponto_pouso = Point(cfg.GOAL_LOC)

    lotes_gdf['altura_z'] = 0
    for idx, lote in lotes_gdf.iterrows():
        if np.random.random() <= cfg.DENSIDADE_PREDIOS:
            if lote.geometry.distance(ponto_inicio) < cfg.ZONA_LIVRE_INICIO or lote.geometry.distance(ponto_pouso) < cfg.ZONA_LIVRE_POUSO:
                lotes_gdf.at[idx, 'altura_z'] = 0
            else:
                lotes_gdf.at[idx, 'altura_z'] = np.random.randint(cfg.ALTURA_MIN, cfg.ALTURA_MAX)

    return lotes_gdf, max_x, max_y