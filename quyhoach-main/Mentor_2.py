import math
import csv
import Node
import MENTOR
import networkx as nx
import matplotlib.pyplot as plt

def prim_dijkstra_backbone_links(ListPosition, backbone_nodes, alpha):
    import heapq

    print("=== BẮT ĐẦU THUẬT TOÁN PRIM-DIJKSTRA ===")
    node_map = {n.get_name(): n for n in ListPosition}
    backbone_names = [n.get_name() for n in backbone_nodes]

    # 1. Tạo đồ thị gốc đầy đủ (G_full)
    G_full = nx.Graph()
    for i in range(len(ListPosition)):
        for j in range(i + 1, len(ListPosition)):
            n1, n2 = ListPosition[i], ListPosition[j]
            cost = math.sqrt((n1.get_position_x() - n2.get_position_x())**2 +
                             (n1.get_position_y() - n2.get_position_y())**2)
            G_full.add_edge(n1.get_name(), n2.get_name(), weight=cost)

    root = backbone_names[0]
    print(f"Khởi tạo gốc: {root}")
    in_tree = set([root])
    tree_edges = []

    # Tính Dijkstra từ gốc
    dijkstra_lengths = nx.single_source_dijkstra_path_length(G_full, root, weight='weight')

    # Heap: (α * dist(root, u) + euclid(u, v), u, v)
    heap = []
    for u in in_tree:
        for v in backbone_names:
            if v in in_tree or u == v:
                continue
            euclid = math.sqrt((node_map[u].get_position_x() - node_map[v].get_position_x())**2 +
                               (node_map[u].get_position_y() - node_map[v].get_position_y())**2)
            cost = alpha * dijkstra_lengths.get(u, float('inf')) + euclid
            heapq.heappush(heap, (cost, u, v))
            print(f"Khởi tạo cạnh ({u}, {v}) với nhãn: α*{dijkstra_lengths.get(u, float('inf')):.2f} + {euclid:.2f} = {cost:.2f}")

    while len(in_tree) < len(backbone_names):
        while True:
            cost, u, v = heapq.heappop(heap)
            if v not in in_tree:
                break
        print(f"Chọn cạnh ({u}, {v}) với nhãn {cost:.2f} để thêm vào cây.")
        tree_edges.append((u, v))
        in_tree.add(v)

        for w in backbone_names:
            if w in in_tree or w == v:
                continue
            euclid = math.sqrt((node_map[v].get_position_x() - node_map[w].get_position_x())**2 +
                               (node_map[v].get_position_y() - node_map[w].get_position_y())**2)
            d_v = nx.dijkstra_path_length(G_full, root, v, weight='weight')
            cost = alpha * d_v + euclid
            heapq.heappush(heap, (cost, v, w))
            print(f"  Cập nhật cạnh ({v}, {w}) với nhãn: α*{d_v:.2f} + {euclid:.2f} = {cost:.2f}")

    print("=== KẾT THÚC THUẬT TOÁN PRIM-DIJKSTRA ===")
    links = [(node_map[u], node_map[v]) for u, v in tree_edges]
    return links


def Mentor2_ISP(ListPosition, TrafficMatrix, MAX, C, w, RadiusRatio, Limit, umin=0.8, alpha=0.2, debug=False):
    # 1. Tính Mentor 1 để lấy các nhóm backbone và truy nhập
    ListMentor = MENTOR.MenTor([Node_copy(n) for n in ListPosition], TrafficMatrix, MAX, C, w, RadiusRatio, Limit, debug)

    # 2. Lấy ra các backbone node (node đầu tiên mỗi group)
    backbone_nodes = [group[0] for group in ListMentor if len(group) > 0]
    backbone_names = [n.get_name() for n in backbone_nodes]

    # 3. Xây dựng các liên kết backbone theo Prim-Dijkstra
    backbone_links = prim_dijkstra_backbone_links(ListPosition, backbone_nodes,alpha)

    # Tạo đồ thị backbone từ các liên kết backbone
    backbone_graph = nx.Graph()
    for n1, n2 in backbone_links:
        backbone_graph.add_edge(n1.get_name(), n2.get_name())

    link_usage = []
    link_cost = []
    link_cost_changed = []
    link_path_count = []

    for n1, n2 in backbone_links:
        name1, name2 = n1.get_name(), n2.get_name()
        cost = math.sqrt((n1.get_position_x() - n2.get_position_x())**2 + (n1.get_position_y() - n2.get_position_y())**2)
        # truyền backbone_graph vào
        usage, count = calc_link_usage(name1, name2, backbone_names, TrafficMatrix, backbone_graph, ListMentor)
        utilization = usage / C if C > 0 else 0
        if utilization < umin:
            cost_new = cost * (1 + alpha)
        else:
            cost_new = cost
        link_usage.append(utilization)
        link_cost.append(cost)
        link_cost_changed.append(cost_new)
        link_path_count.append(count)

    return backbone_names, link_path_count, link_cost, link_cost_changed, link_usage, ListMentor

def find_backbone_of_node(node_id, ListMentor):
    for group in ListMentor:
        if node_id in [n.get_name() for n in group]:
            return group[0].get_name()
    return None

def calc_link_usage(name1, name2, backbone_names, TrafficMatrix, backbone_graph, ListMentor):
    usage = 0
    count = 0
    n = len(TrafficMatrix)
    for src in range(1, n+1):
        for dst in range(1, n+1):
            if src == dst:
                continue
            flow = TrafficMatrix[src-1][dst-1]
            if flow > 0:
                b_src = find_backbone_of_node(src, ListMentor)
                b_dst = find_backbone_of_node(dst, ListMentor)
                if b_src is None or b_dst is None or b_src == b_dst:
                    continue
                try:
                    path = nx.shortest_path(backbone_graph, source=b_src, target=b_dst)
                    for i in range(len(path)-1):
                        u, v = path[i], path[i+1]
                        if (u == name1 and v == name2) or (u == name2 and v == name1):
                            usage += flow
                            count += 1
                            break
                except nx.NetworkXNoPath:
                    continue
    return usage, count

def write_result(filename, backbone, link_path_count, link_cost, link_cost_changed, link_usage):
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("Liên kết\tSố đường\tĐộ sử dụng\tGiá ban đầu\tGiá thay đổi\n")
        for i in range(len(link_path_count)):
            f.write(f"{backbone[i]}-{backbone[(i+1)%len(backbone)]}\t{link_path_count[i]}\t{link_usage[i]:.2f}\t{link_cost[i]:.2f}\t{link_cost_changed[i]:.2f}\n")

def plot_backbone(ListPosition,_list_mentor, backbone_links, MAX):
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'magenta', 'lime', 'brown']
    num_colors = len(colors)

    for group_idx, group in enumerate(_list_mentor):
        if not group:
            continue

        color = colors[group_idx % num_colors]

        xpos = []
        ypos = []

        for node in group:
            xpos.append(node.get_position_x())
            ypos.append(node.get_position_y())

        # --- Backbone Node ---
        backbone = group[0]
        plt.plot(backbone.get_position_x(), backbone.get_position_y(),
                 'o', markersize=14, markerfacecolor=color, markeredgecolor='black', markeredgewidth=2, zorder=3)
        plt.text(backbone.get_position_x(), backbone.get_position_y(), str(backbone.get_name()),
                 color='white', size=12, weight='bold', ha="center", va="center",
                 bbox=dict(facecolor=color, edgecolor='black', boxstyle='round'), zorder = 3)

        # --- Access Nodes ---
        for node in group[1:]:
            plt.plot(node.get_position_x(), node.get_position_y(),
                     'o', markersize=7, markerfacecolor=color, markeredgecolor='black', markeredgewidth=1, zorder = 2)
            plt.text(node.get_position_x(), node.get_position_y(), str(node.get_name()),
                     color='black', size=9, ha="center", va="center",
                     bbox=dict(facecolor=color, edgecolor='gray', boxstyle='round'), zorder = 2)

    # Thiết lập vùng hiển thị
    
    # Vẽ liên kết backbone theo cây Prim-Dijkstra
    for n1, n2 in backbone_links:
        plt.plot([n1.get_position_x(), n2.get_position_x()],
                 [n1.get_position_y(), n2.get_position_y()], 'k-', linewidth=2)
    plt_margin = MAX * 0.05
    plt.axis([-plt_margin, MAX + plt_margin, -plt_margin, MAX + plt_margin])
    plt.title("Cây Prim-Dijkstra các nút backbone", fontsize=14)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()

def Node_copy(node):
    # Deep copy cho Node tối giản
    n = Node.Node()
    n.create_name(node.get_name())
    n.set_position(node.get_position_x(), node.get_position_y())
    n.set_traffic(node.get_traffic())
    return n

# Không nên để đoạn này trong Mentor_2.py:
# backbone, link_path_count, link_cost, link_cost_changed, link_usage = Mentor2_ISP(
#     ListPosition, TrafficMatrix, MAX, C, w, RadiusRatio, Limit
# )
# write_result('mentor2_result.txt', backbone, link_path_count, link_cost, link_cost_changed, link_usage)
