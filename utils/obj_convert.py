def parse_obj(filename: str) -> list[list[float]]:
    all_lines = list()
    # read all lines
    with open(filename, 'r') as f:
        line = f.readline()
        while len(line) > 0:
            all_lines.append(line)
            line = f.readline()
    all_lines_split = list(map(lambda x: x.split(" "), all_lines))
    vertex_strs = list(filter(lambda x: x[0] == 'v', all_lines_split))
    facet_strs = list(filter(lambda x: x[0] == 'f', all_lines_split))

    facets = []
    for f in facet_strs:
        v0_idx = int(f[1].split("/")[0])
        v1_idx = int(f[2].split("/")[0])
        v2_idx = int(f[3].split("/")[0])

        v0_coord = list(map(lambda x: float(x), vertex_strs[v0_idx][1:]))
        v1_coord = list(map(lambda x: float(x), vertex_strs[v0_idx][1:]))
        v2_coord = list(map(lambda x: float(x), vertex_strs[v0_idx][1:]))
        facets.append(v0_coord + v1_coord + v2_coord)
    return facets


def dispatch_material(vertices: list[list[float]], color: list[float], illu: float = 600., ref_type: int = 0x02,
                      n: float = 1.0) -> list[dict]:
    res = []
    for v in vertices:
        d = {"vertex": v, "color": color, "illu": illu, "ref_type": ref_type, "n": n}
        res.append(d)
    return res
