import numpy as np
import cv2
import copy
from jenks_natural_breaks import classify
import networkx as nx
import matplotlib.pyplot as plt
from Variables import Node_voronoi
from std_msgs.msg import String

def get_voronoi(q_NN, q_m, q_r, q_f, q_env, p, sp):
    origin_x = q_NN['origin_x']
    origin_y = q_NN['origin_y']
    vertices = q_NN['vertices']

    # get node/edge pos info from segments
    pos_nodes = []
    edges_pos = []
    for i_v, v in enumerate(vertices):
        pos_0 = (v.path[0].x + origin_x, v.path[0].y + origin_y)
        pos_1 = (v.path[-1].x + origin_x, v.path[-1].y + origin_y)
        if pos_0 not in pos_nodes:
            pos_nodes.append(pos_0)
        if pos_1 not in pos_nodes:
            pos_nodes.append(pos_1)
        edges_pos.append((pos_0, pos_1))

    # get isrobot node
    dist = np.inf
    node_isrobot = 0
    for i_n, pos in enumerate(pos_nodes):
        x = pos[0]
        y = pos[1]
        cand_dist = get_dist((x, y), (q_r['pos'][0], q_r['pos'][1]))
        if cand_dist < dist:
            node_isrobot = i_n
            dist = cand_dist

    # generate nodes
    nodes = []
    for i_n, pos_node in enumerate(pos_nodes):
        isrobot = False
        if i_n == node_isrobot:
            isrobot = True
        node = Node_voronoi(index=i_n, pos=(pos_node[0], pos_node[1]), to_go=False, isrobot=isrobot, value=-1, type=-1)
        nodes.append(node)

    # generate edges
    edges_idx = []
    for i, pos_edge in enumerate(edges_pos):
        idx_0 = [i for i, node in enumerate(pos_nodes) if node == pos_edge[0]][0]
        idx_1 = [i for i, node in enumerate(pos_nodes) if node == pos_edge[1]][0]
        edges_idx.append((idx_0, idx_1))

    # graph construction
    graph = nx.Graph()
    for node in nodes:
        graph.add_node(node.index, pos=node.pos, to_go=node.to_go, isrobot=node.isrobot, value=-1, type=-1)
    graph.add_edges_from(edges_idx)
    q_NN['global_network'] = graph

    # # node 'to_go' property set
    # q_NN = validate_to_go_itself(q_NN, q_m, p)

    q_NN['global_network'] = cutoff_isolated_subnet(q_NN['global_network'])
    q_NN['global_network'] = cutoff_dup_nodes(q_NN['global_network'])
    q_NN['global_network'], mapping = relabel_network(q_NN['global_network'])

    q_NN['global_network'] = assign_category_to_graph(q_NN, q_m, q_env, sp)
    q_NN['global_network'] = reduce_graph(q_NN['global_network'], sp)

    q_NN['global_network'] = validate_to_go(q_NN['global_network'], q_m, p)

    pos = nx.get_node_attributes(q_NN['global_network'], 'pos')
    labels = nx.get_node_attributes(q_NN['global_network'], 'pos')
    nx.draw(q_NN['global_network'], pos, labels=labels)
    plt.show()

    # # draw plt
    # pos = nx.get_node_attributes(q_NN['global_network'], 'pos')
    # labels = nx.get_node_attributes(q_NN['global_network'], 'category_str')
    # nx.draw(q_NN['global_network'], pos, labels=labels)
    # plt.show()
    # nx.write_gpickle(q_NN['global_network'], "test.gpickle")
    #
    # q_NN['global_network']= reduce_graph(q_NN['global_network'], p)
    #
    # # draw plt
    # pos = nx.get_node_attributes(q_NN['global_network'], 'pos')
    # labels = nx.get_node_attributes(q_NN['global_network'], 'category_str')
    # nx.draw(q_NN['global_network'], pos, labels=labels)
    # plt.show()
    # nx.write_gpickle(q_NN['global_network'], "test.gpickle")

    return q_NN, q_m, q_r, p, q_f

def validate_to_go(NN, q_m, p):
    graph = copy.deepcopy(NN)
    scale_per = p['scale_per']
    map_2d_1max_scaled = scale_map(q_m['map_2d'], p, scale_percent=scale_per)
    r_scaled = int(np.round(p['r'] / (q_m['res'] * p['scale_per'])))

    to_gos = [x for x, y in graph.nodes(data=True) if 1]

    num = 0
    for to_go in to_gos:
        pos_node = graph.nodes[to_go]['pos']
        x = pos_node[0]
        y = pos_node[1]
        x_pix = int(np.round((x - q_m['origin'][0]) // q_m['res']))
        y_pix = int(np.round((y - q_m['origin'][1]) // q_m['res']))
        x_pix_scaled = int(x_pix * scale_per / 100.)
        y_pix_scaled = int(y_pix * scale_per / 100.)

        loc_map, loc_pos = get_local_map_arbitrary2(map_2d_1max_scaled, (x_pix_scaled, y_pix_scaled), r_scaled-5)

        candidate = False
        for (j,i) in ((s0, s1) for s0 in range(0,loc_map.shape[0]) for s1 in range(0,loc_map.shape[1])):
            dist = get_dist(loc_pos, (i,j))
            if (dist <= r_scaled-5) and visible(loc_map, loc_pos, (i,j)):
                if (loc_map[j,i] ==  0.5):
                    candidate = True
                    num += 1
                    break
        if candidate:
            graph.nodes[to_go]['to_go'] = True
        else:
            graph.nodes[to_go]['to_go'] = False

    return graph

def get_local_map_arbitrary2(glo_map, glo_pos, r=20):
    (x, y) = glo_pos
    lx, ux = max(0, x - r), min(glo_map.shape[1], x + r + 1)
    ly, uy = max(0, y - r), min(glo_map.shape[0], y + r + 1)

    loc_map = copy.deepcopy(glo_map[ly:uy, lx:ux])
    x = min(x, r)
    y = min(y, r)
    # for j in range(loc_map.shape[0]):
    #     for i in range(loc_map.shape[1]):
    #         dist = get_dist((x, y), (i, j))
    #         if (dist > r):
    #             loc_map[j, i] = 0.5
    return loc_map, (x, y)

def get_local_map_arbitrary(glo_map, glo_pos, r=20):
    (x, y) = glo_pos
    lx, ux = max(0, x - r - 1), min(glo_map.shape[1], x + r + 1 + 1)
    ly, uy = max(0, y - r - 1), min(glo_map.shape[0], y + r + 1 + 1)

    loc_map = copy.deepcopy(glo_map[ly:uy, lx:ux])
    x = min(x, r+1)
    y = min(y, r+1)
    for j in range(loc_map.shape[0]):
        for i in range(loc_map.shape[1]):
            dist = get_dist((x, y), (i, j))
            if (dist <= r):
                if not visible(loc_map, (x, y), (i, j)) and loc_map[j,i]!=0:
                    loc_map[j, i] = 0.5
            else:
                loc_map[j, i] = 0.5
    return loc_map, (x, y)

def assign_category_to_graph(q_NN, q_m, q_env, sp):
    graph = copy.deepcopy(q_NN['global_network'])
    floorplan_img = copy.deepcopy(q_env.GT_map)
    cmap_floorplan = sp['cmap_floorplan']

    idx_nodes = [x for x, y in graph.nodes(data=True) if 1]
    for i_n in idx_nodes:
        pos_node_cart = graph.nodes[i_n]['pos']
        pos_node_img = convert_pos_cart2img(pos_cart=pos_node_cart, res=q_m['res'], dim_img=np.shape(floorplan_img))
        rgb_node_list = floorplan_img[pos_node_img[1], pos_node_img[0], :]
        rgb_node = (rgb_node_list[0], rgb_node_list[1], rgb_node_list[2])

        for category, colormaps in cmap_floorplan.items():
            if rgb_node in colormaps:
                graph.nodes[i_n]['type_str'] = category
                graph.nodes[i_n]['type'] = sp['imap_category'][category]
                break
    return graph

def convert_pos_cart2img(pos_cart, res, dim_img):
    x = pos_cart[0]
    y = pos_cart[1]

    pos_img_x = int(np.floor(x/res + dim_img[1]/2.))
    pos_img_y = int(np.floor(y/res + dim_img[0]/2.))

    return (pos_img_x, pos_img_y)


def get_dist(start, end):
    (x0, y0), (x1, y1) = start, end
    return np.sqrt((x1-x0)**2 +  (y1-y0)**2)

def bresenham(start, end):
    (x0, y0), (x1, y1) = start, end
    dx, dy = x1-x0, y1-y0
    is_steep = abs(dy) > abs(dx)
    if is_steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    swapped = False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped = True
    dx, dy = x1-x0, y1-y0
    error = int(dx/2.0)
    y_step = 1 if y0 < y1 else -1
    y = y0
    points = []
    for x in range(x0, x1+1):
        coord = (y,x) if is_steep else (x,y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:
        points.reverse()
    return points

def visible(all_map, start, end):
    """
    Returns bool on visibility of a cell (i.e., status of path from start to end, less end)
    """
    for x, y in bresenham(start, end)[:-1]:
        if (all_map[y,x] <= 0.5):
            return False
    return True

def get_visibility_map(q_m, p):
    loc_map = copy.deepcopy(q_m['map_2d_local_1max_scaled'])
    r = p['r_pix_scaled']

    vis_map = np.zeros(loc_map.shape)
    f = (loc_map == 1).sum()
    ys, xs = np.where(loc_map == 1)
    for y, x in zip(ys, xs):
        for j in range(loc_map.shape[0]):
            for i in range(loc_map.shape[1]):
                dist = get_dist((x, y), (i, j))
                if (dist <= r):
                    if visible(loc_map, (x, y), (i, j)):
                        vis_map[y, x] += np.floor(loc_map[j, i]) / f
    q_m['vis_map'] = vis_map
    return q_m

def get_classification(q_m, p):
    vis_map = copy.deepcopy(q_m['vis_map'])
    levels = p['N_c']
    vis = vis_map.flatten()
    vis = vis[vis != 0]
    breaks = classify(data=vis, n_classes=levels)

    class_map = np.zeros(vis_map.shape)
    for j in range(vis_map.shape[0]):
        for i in range(vis_map.shape[1]):
            val = vis_map[j,i]
            b = 1
            if val >= breaks[0]:
                while True:
                    if (val <= breaks[b]):
                        class_map[j,i] = b
                        break
                    b += 1
    q_m['class_map'] = class_map
    return q_m

def candidate_to_go(neighbors):
    to_go = False if -1 not in neighbors else True
    return to_go

def get_node_type(centroid, neighbors_centroids):
    if neighbors_centroids:
        neighbors_lvl = [x for x in (y.level for y in neighbors_centroids)]
        if centroid.level < np.min(neighbors_lvl):
            return 'pit'
        elif centroid.level > np.max(neighbors_lvl):
            return 'peak'
        else:
            return 'pass'

def loc2glo(pos, glo, loc):
    diff = tuple(map(lambda i, j: i - j, pos, loc))
    return tuple(map(lambda i, j: i + j, glo, diff))

def get_key(dict, val):
    keys = []
    for key, value in dict.items():
        if val == value:
            keys.append(key)
    return keys

def cutoff_isolated_subnet(network):
    subgraphs = list(nx.connected_components(network))
    isrobot = get_key(nx.get_node_attributes(network, 'isrobot'),1)[0]
    for i_s in range(0,len(subgraphs)):
        if isrobot in subgraphs[i_s]:
            continue
        else:
            for i_n in subgraphs[i_s]:
                network.remove_node(i_n)
    return network

def cutoff_dup_nodes(network):
    pos_dict = nx.get_node_attributes(network, 'pos')

    reverse_dict = {}
    for key, value in pos_dict.items():
        try:reverse_dict[value].append(key)
        except:reverse_dict[value] = [key]
    dup_pairs = [values for key, values in reverse_dict.items() if len(values) > 1]

    robot_node = get_key(nx.get_node_attributes(network, 'isrobot'),1)[0]
    for i_p, nodes in enumerate(dup_pairs):
        if robot_node in nodes:
            nodes.remove(robot_node)
        else:
            nodes.remove(max(nodes))
        for i_n, node in enumerate(nodes):
            network.remove_node(node)
    return network

def relabel_network(network):
    #remap network just in case there is empty index
    mapping = {}
    index = 0
    for n in network.nodes():
        mapping[n] = index
        index += 1
    network = nx.relabel_nodes(network, mapping, copy=True)
    return network, mapping

def get_local_map(q_r, q_m, p):
    glo_pos = (q_r['x_pix_scaled'], q_r['y_pix_scaled'])
    glo_map = q_m['map_2d_1max_scaled']
    r = p['r_pix_scaled']

    (x, y) = glo_pos
    lx, ux = max(0, x-r), min(glo_map.shape[1], x+r+1)
    ly, uy = max(0, y-r), min(glo_map.shape[0], y+r+1)
    loc_map = copy.deepcopy(glo_map[ly:uy, lx:ux])
    x = min(x, r)
    y = min(y, r)
    for j in range(loc_map.shape[0]):
        for i in range(loc_map.shape[1]):
            dist = get_dist((x,y), (i,j))
            if (dist <= r):
                if not visible(loc_map, (x,y), (i,j)):
                    loc_map[j,i] = 0.5
            else:
                loc_map[j,i] = 0.5

    q_m['map_2d_local_1max_scaled'] = loc_map
    q_r['x_pix_local_scaled'] = x
    q_r['y_pix_local_scaled'] = y

    return q_m

def scale_map(map, p, scale_percent=100):
    unknown_mask = np.zeros(np.shape(map))
    free_mask = np.zeros(np.shape(map))
    occ_mask = np.zeros(np.shape(map))

    idx_unknown = np.where(map == -1)
    idx_free = np.where(map == 0)
    idx_occ = np.where(map == 100)

    unknown_mask[idx_unknown] = 1.
    free_mask[idx_free] = 1.
    occ_mask[idx_occ] = 1.

    width = int(map.shape[1] * scale_percent / 100)
    height = int(map.shape[0] * scale_percent / 100)
    dim = (width, height)

    unknown_mask_resized = cv2.resize(unknown_mask, dim, interpolation=cv2.INTER_AREA)
    free_mask_resized = cv2.resize(free_mask, dim, interpolation=cv2.INTER_AREA)
    occ_mask_resized = cv2.resize(occ_mask, dim, interpolation=cv2.INTER_AREA)
    unknown_mask_resized = (unknown_mask_resized > 0.5).astype(float)
    free_mask_resized = (free_mask_resized > 0.5).astype(float)
    occ_mask_resized = (occ_mask_resized > 0.000001).astype(float)

    map_resized = np.zeros(dim)

    for i, x in np.ndenumerate(map_resized):
        is_unknown = unknown_mask_resized[i]
        is_free = free_mask_resized[i]
        is_occ = occ_mask_resized[i]
        if is_occ == 1:
            map_resized[i] = p['def_occ']
        elif is_unknown == 1:
            map_resized[i] = p['def_unknown']
        else:
            map_resized[i] = p['def_free']

    return map_resized

def scale_data(q_m, q_r, p):
    scale_per = p['scale_per']
    map_2d_1max_scaled = scale_map(q_m['map_2d'], p, scale_percent=scale_per)
    robot_pos_scaled = (int(q_r['pos'][0]*scale_per/100.), int(q_r['pos'][1]*scale_per/100.))
    sensor_range_scaled = int(np.round(p['r'] / (q_m['res'] * p['scale_per'])))
    return map_2d_1max_scaled, robot_pos_scaled, sensor_range_scaled


def gcd(a, b):
    if b > a:
        tmp = a
        a = b
        b = tmp
    while b > 0:
        c = b
        b = a % b
        a = c
    return a


def lcm(a, b):
    return a * b // gcd(a, b)


def contract(g):
    """
    Contract chains of neighbouring vertices with degree 2 into one hypernode.
    Arguments:
    ----------
    g -- networkx.Graph instance
    Returns:
    --------
    h -- networkx.Graph instance
        the contracted graph
    hypernode_to_nodes -- dict: int hypernode -> [v1, v2, ..., vn]
        dictionary mapping hypernodes to nodes
    """

    # create subgraph of all nodes with degree 2
    is_chain = [node for node, degree in g.degree() if degree == 2]
    chains = g.subgraph(is_chain)

    # contract connected components (which should be chains of variable length) into single node
    components = list(nx.components.connected_component_subgraphs(chains))
    hypernode = max(g.nodes()) +1
    hypernodes = []
    hyperedges = []
    hypernode_to_nodes = dict()
    false_alarms = []
    for component in components:
        if component.number_of_nodes() > 1:

            hypernodes.append(hypernode)
            vs = [node for node in component.nodes()]
            hypernode_to_nodes[hypernode] = vs

            # create new edges from the neighbours of the chain ends to the hypernode
            component_edges = [e for e in component.edges()]
            for v, w in [e for e in g.edges(vs) if not ((e in component_edges) or (e[::-1] in component_edges))]:
                if v in component:
                    hyperedges.append([hypernode, w])
                else:
                    hyperedges.append([v, hypernode])

            hypernode += 1

        else: # nothing to collapse as there is only a single node in component:
            false_alarms.extend([node for node in component.nodes()])

    # initialise new graph with all other nodes
    not_chain = [node for node in g.nodes() if not node in is_chain]
    h = g.subgraph(not_chain + false_alarms)
    h.add_nodes_from(hypernodes)
    h.add_edges_from(hyperedges)

    return h, hypernode_to_nodes

def reduce_graph(G_in, sp):
    G = copy.deepcopy(G_in)
    list_categories = sp['imap_category'].keys()

    for category in list_categories:
        nodes_c = [node for node, data in G.nodes(data=True) if data.get("type_str") == category]

        G_c = nx.subgraph(G, nodes_c)
        G_c_s_list = list(nx.connected_component_subgraphs(G_c))

        for i_G, G_c_s in enumerate(G_c_s_list):
            if nx.number_of_nodes(G_c_s) > 8:
                centrality = nx.edge_betweenness_centrality(G_c_s)

                edges_cycle = []
                try:
                    edges_cycle = nx.find_cycle(G_c_s)
                except:
                    pass
                edges_acycle = list(G_c_s.edges() - edges_cycle)
                centrality_acycle = {k:v for (k, v) in centrality.items() if (k in edges_acycle) and v>0}


                edge_center = max(centrality_acycle.keys(), key=(lambda k:centrality_acycle[k]))
                G_c_s.remove_edge(edge_center[0], edge_center[1])

                G_c_s_list.remove(G_c_s)
                G_c_s_list.extend(list(nx.connected_component_subgraphs(G_c_s)))

        for G_c_s in G_c_s_list:
            nodes_i = list(G_c_s.nodes())
            isrobot_i = {k: v for k, v in nx.get_node_attributes(G_c_s, 'isrobot').items() if k in nodes_i}
            to_go_i = {k: v for k, v in nx.get_node_attributes(G_c_s, 'to_go').items() if k in nodes_i}

            to_go = any(to_go_i.values())
            isrobot_node = [k for k, v in isrobot_i.items() if v == True]
            if len(isrobot_node) == 0:
                poses_x = [G.node[nodes_i[0]]['pos'][0]]
                poses_y = [G.node[nodes_i[0]]['pos'][1]]
                for node in nodes_i[1:]:
                    poses_x.append(G.node[node]['pos'][0])
                    poses_y.append(G.node[node]['pos'][1])
                    G = nx.contracted_nodes(G, nodes_i[0], node)
                    G.node[nodes_i[0]]['to_go'] = to_go
                G.node[nodes_i[0]]['pos'] = (sum(poses_x)/len(poses_x), sum(poses_y)/len(poses_y))
            else:
                nodes_i_wo_isrobot = [n for n in nodes_i if n not in isrobot_node]
                poses_x = [G.node[isrobot_node[0]]['pos'][0]]
                poses_y = [G.node[isrobot_node[0]]['pos'][1]]
                for node in nodes_i_wo_isrobot:
                    poses_x.append(G.node[node]['pos'][0])
                    poses_y.append(G.node[node]['pos'][1])
                    G = nx.contracted_nodes(G, isrobot_node[0], node)
                    G.node[isrobot_node[0]]['to_go'] = to_go
                G.node[isrobot_node[0]]['pos'] = (sum(poses_x)/len(poses_x), sum(poses_y)/len(poses_y))

    category = nx.get_node_attributes(G, 'type_str')
    idx_pass = [k for k,v in category.items() if v=='pass']

    for node in idx_pass:
        node_neighbors = list(nx.neighbors(G, node))
        node_nonpass = [n for n in node_neighbors if G.nodes[n]['type_str'] != 'pass'][0]
        G = nx.contracted_nodes(G, node_nonpass, node)
    return G

def get_dict_NN_for_FVE(G):
    # convert NN to dict
    dict_G = {}
    E = [[v1, v2] for (v1, v2) in list(nx.edges(G))]
    C = {str(n): str(c) for n, c in nx.get_node_attributes(G, 'type').items()}
    dict_G["edges"] = E
    dict_G["features"] = C

    return dict_G

def get_nodes_FVE(NN, p, srv_pred_FVE):
    # jsonstr encoded navigation network
    NN_jsonstr = get_dict_NN_for_FVE(NN)
    msg_NN_jsonstr = String()
    msg_NN_jsonstr.data = str(NN_jsonstr)

    # intereset node
    to_go_dict = nx.get_node_attributes(NN, 'to_go')

    # [1xN_c] target existence probability vector
    msg_p_E = String()
    msg_p_E.data = str(p.p_E)

    FVE = {}
    for v, to_go in to_go_dict.items():
        if to_go == True:
            FVE_v = srv_pred_FVE(msg_NN_jsonstr, v, msg_p_E, p.N_c, p.M, p.K, p.gamma)
        else:
            FVE_v = 0
        FVE[v] = FVE_v

    return FVE

def get_jsonstr_graph_for_planner(NN):
    edges = list(NN.edges())
    value = nx.get_node_attributes(NN, 'value')
    pos = nx.get_node_attributes(NN, 'pos')
    isrobot = nx.get_node_attributes(NN, 'isrobot')
    to_go = nx.get_node_attributes(NN, 'to_go')
    G_json = {"edges": edges, "value": value, "isrobot": isrobot, "pos": pos, "to_go": to_go}
    G_json_str = str(G_json)

    return G_json_str

def get_path(map_msg, NN, srv_get_path_GEST):
    G_json_str = get_jsonstr_graph_for_planner(NN)
    msg_NN_jsonstr = String()
    msg_NN_jsonstr.data = G_json_str

    output = srv_get_path_GEST(map_msg, msg_NN_jsonstr, [])
    path_msg = output.path
    path = [(point.pose.position.x, point.pose.position.y) for point in path_msg.poses]

    return path

