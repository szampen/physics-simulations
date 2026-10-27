"""
Collision detection algorithms: Brute Force, Sweep and Prune, Bounding Volume Hierarchy.

Object representation: AABB (Axis-Aligned Bounding Box) = (minx, miny, minz, maxx, maxy, maxz).

Every algorithm returns: (set of pairs {(i, j) : i < j}, number of AABB checks)
"""


# ---------------------------------------------------------------- Brute Force
def brute_force(boxes):
    pairs = set()
    checks = 0
    n = len(boxes)
    for i in range(n):
        ax0, ay0, az0, ax1, ay1, az1 = boxes[i]
        for j in range(i + 1, n):
            bx0, by0, bz0, bx1, by1, bz1 = boxes[j]
            checks += 1
            if (ax0 <= bx1 and ax1 >= bx0 and ay0 <= by1 and ay1 >= by0
                    and az0 <= bz1 and az1 >= bz0):
                pairs.add((i, j))
    return pairs, checks


# ------------------------------------------------------------ Sweep and Prune
def sweep_and_prune(boxes):
    """
    *) Sorting by min_X
    *) For every object, we check the next ones, until min_X <= max_X of current object
        (Intervals on axis X overlap)
    *) Then, we check the Y's and Z's
    """
    pairs = set()
    checks = 0
    order = sorted(range(len(boxes)), key=lambda k: boxes[k][0])
    n = len(order)
    for a in range(n):
        i = order[a]
        ax0, ay0, az0, ax1, ay1, az1 = boxes[i]
        for b in range(a + 1, n):
            j = order[b]
            bx0, by0, bz0, bx1, by1, bz1 = boxes[j]
            if bx0 > ax1:  # end of interval on X axis ==> stop
                break
            checks += 1
            if ay0 <= by1 and ay1 >= by0 and az0 <= bz1 and az1 >= bz0:
                pairs.add((i, j) if i < j else (j, i))
    return pairs, checks


# ------------------------------------------------------------------------ BVH
def expand_bits(v):
    """Places two zeros between every bit of 10-bit number (for Morton 3d code)"""
    v = (v * 0x00010001) & 0xFF0000FF
    v = (v * 0x00000101) & 0x0F00F00F
    v = (v * 0x00000011) & 0xC30C30C3
    v = (v * 0x00000005) & 0x49249249
    return v


def morton3d(x, y, z, world_size=100.0):
    """
    Calculates Morton code for coordinates in world [-world_size/2, world_size/2]^3

    1. Normalization to [0, 1]
    2. Narrowing the coordinates to [0, 1] interval
    3. Scaling to [0, 1023] (10-bit coding) - dividing the world into 1024 x 1024 x 1024 little cubes
    4. Placing zeros between bits (expand_bits)
    5. Interleaving:
        zz << 2:    Z  0  0  Z  0  0  Z  0  0  ...
        yy << 1:    0  Y  0  0  Y  0  0  Y  0  ...
        xx:         0  0  X  0  0  X  0  0  X  ...
        ---------------------------------------------------
        Result:     Z  Y  X  Z  Y  X  Z  Y  X  ... (30 bits)
    """

    # Step 1
    def normalize_coord(val):
        return (val + world_size / 2) / world_size

    x = normalize_coord(x)
    y = normalize_coord(y)
    z = normalize_coord(z)

    # Step 2
    x = max(0.0, min(x, 1.0))
    y = max(0.0, min(y, 1.0))
    z = max(0.0, min(z, 1.0))

    # Step 3
    x = min(int(x * 1023), 1023)
    y = min(int(y * 1023), 1023)
    z = min(int(z * 1023), 1023)

    # Step 4
    xx = expand_bits(x)
    yy = expand_bits(y)
    zz = expand_bits(z)

    # Step 5
    return xx | (yy << 1) | (zz << 2)


def build_bvh(boxes, world_size=100.0):
    """
    Builds BVH tree:
    Sorting by Morton code and dividing in the middle
    Node = [min_X, min_Y, min_Z, max_X, max_Y, max_Z, left, right, leaf_id]

    Returns (node_list, root_index)
    """
    n = len(boxes)

    keyed = []
    for i, b in enumerate(boxes):
        cx, cy, cz = ((b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2) # center of bounding box "b"
        code = morton3d(cx, cy, cz, world_size=world_size) # conversion of box's center to Morton code
        keyed.append((code, i))
    keyed.sort() # ascending by Morton code
    ids = [i for _, i in keyed]  # stores ids, sorted order of original objects

    nodes = []

    def make(begin, end):  # closed interval [begin, end]
        # 1) leaf - only one object left
        if begin == end:
            i = ids[begin]
            nodes.append(list(boxes[i]) + [-1, -1, i])
            return len(nodes) - 1 # returns index of leaf

        # 2) recursive case - dividing in half
        m = (begin + end) // 2
        l = make(begin, m) # left subtree
        r = make(m + 1, end) # right subtree

        # 3) Bounding Box of the parent
        a, b = nodes[l], nodes[r] # bounding boxes of children

        # new bounding box that include both of the children
        nodes.append([min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2]),
                      max(a[3], b[3]), max(a[4], b[4]), max(a[5], b[5]),
                      l, r, -1])
        return len(nodes) - 1

    root = make(0, n - 1)
    return nodes, root


def bvh_query(boxes, nodes, root):
    """
    Check all collisions using BVH
    """
    pairs = set()
    checks = 0
    for i, (x0, y0, z0, x1, y1, z1) in enumerate(boxes):
        # depth search - starting with root
        stack = [root]
        while stack:
            nd = nodes[stack.pop()] # extracting id from stack and retrieving data about the node from nodes list
            checks += 1
            # if there is not any overlap, we skip the entire subtree, so we do not add its children on stack
            if not (x0 <= nd[3] and x1 >= nd[0] and y0 <= nd[4] and y1 >= nd[1]
                    and z0 <= nd[5] and z1 >= nd[2]):
                continue
            # if there is overlap:

            # leaf node - actual object => adding to collision pairs
            if nd[8] != -1:  # lisc
                if nd[8] > i:  # only j > i (deduplication and prevents from adding itself)
                    pairs.add((i, nd[8]))
            # bounding box - containing two objects with their indicators in "left" and "right" fields
            # adding children to stack
            else:
                stack.append(nd[6])
                stack.append(nd[7])
    return pairs, checks


def bvh(boxes):
    nodes, root = build_bvh(boxes)
    return bvh_query(boxes, nodes, root)


ALGORITHMS = {"Brute Force": brute_force, "Sweep and Prune": sweep_and_prune, "BVH": bvh}