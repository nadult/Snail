#include "vtree.h"
#include <cstring>


VTree::VTree(const DICOM &dic) {
	width  = (dic.width  + Block::size - 1) / Block::size;
	height = (dic.height + Block::size - 1) / Block::size;
	depth  = (dic.depth  + Block::size - 1) / Block::size;

	blocks.resize(width * height * depth);

	int dw = dic.width  % Block::size == 0? Block::size : dic.width  % Block::size;
	int dh = dic.height % Block::size == 0? Block::size : dic.height % Block::size;
	int dd = dic.depth  % Block::size == 0? Block::size : dic.depth  % Block::size;

	for(int z = 0; z < depth; z++)
		for(int y = 0; y < height; y++)
			for(int x = 0; x < width; x++) {
				Block &block = blocks[x + (z * height + y) * width];
				memset(block.data, 0, sizeof(block.data));

				const u16 *src = &dic.data[(x + (z * dic.height + y) * dic.width) * Block::size];
				int tw = x == width  - 1? dw : Block::size;
				int th = y == height - 1? dh : Block::size;
				int td = z == depth  - 1? dd : Block::size;

				for(int tz = 0; tz < td; tz++)
					for(int ty = 0; ty < th; ty++) {
						u16 *dptr = &block.data[(tz * Block::size + ty) * Block::size];
						const u16 *sptr = src + (tz * dic.height + ty) * dic.width;
						for(int tx = 0; tx < tw; tx++)
							dptr[tx] = sptr[tx];
					}
			}

	nodes.resize(blocks.size());
	boxes.resize(blocks.size());

	for(int z = 0; z < depth; z++)
		for(int y = 0; y < height; y++)
			for(int x = 0; x < width; x++) {
		int n = x + (z * height + y) * width;

		Node &node = nodes[n];
		const Block &block = blocks[n];
		node.min = node.max = block.data[0];
		for(uint i = 1; i < Block::size3; i++) {
			node.min = Min(node.min, block.data[i]);
			node.max = Max(node.max, block.data[i]);
		}

		node.leftChild = n;
		node.rightChild = -1;
		node.axis = 3;
		boxes[n].min = Vec3f(x, y, z);
		boxes[n].max = Vec3f(x + Block::size, y + Block::size, z + Block::size);
	}

	BuildTree(width, height, depth, 0, nodes.size());
}

void VTree::BuildTree(int w, int h, int d, int first, int count) {
	printf("%d %d %d\n", w, h, d);
	int axis = h > w? (d > h? 2 : 1) : (d > w? 2 : 0);
				
	//TODO: special case if left node has only one child, and there is no right node

	for(int z = 0; z < d; z += (axis == 2? 2 : 1))
		for(int y = 0; y < h; y += (axis == 1? 2 : 1))
			for(int x = 0; x < w; x += (axis == 0? 2 : 1)) {
				int idx = first + x + (z * h + y) * w;

				Node newNode;
				BBox box;

				if(axis == 0? x == w - 1 : axis == 1? y == h - 1 : z == d - 1) { // single child
					newNode = nodes[idx];
					box = boxes[idx];
				}
				else {
					int nidx = idx + (axis == 0? 1 : axis == 1? w : w * h);

					newNode.leftChild = idx;
					newNode.rightChild = nidx;
					newNode.axis = axis;
					newNode.min = Min(nodes[idx].min, nodes[nidx].min);
					newNode.max = Max(nodes[idx].max, nodes[nidx].max);
					box = boxes[idx];
					newNode.split = (&box.max.x)[axis];
					box += boxes[nidx];
				}

				nodes.push_back(newNode);
				boxes.push_back(box);
			}

	w = axis == 0? (w + 1) / 2 : w;
	h = axis == 1? (h + 1) / 2 : h;
	d = axis == 2? (d + 1) / 2 : d;
	
	int newCount = nodes.size() - first - count;
	InputAssert(w * h * d == newCount);

	if(newCount == 1) {
		startNode = nodes.size() - 1;
		treeBBox = boxes[startNode];
		return;
	}


	BuildTree(w, h, d, first + count, newCount);
}

u16 VTree::Trace(const Vec3f &origin, const Vec3f &dir, u16 cmin) const {
	Vec3f idir = VInv(dir);

	BBox box = GetBBox();
	if(dir.x < 0.0f)
		Swap(box.min.x, box.max.x);
	if(dir.y < 0.0f)
		Swap(box.min.y, box.max.y);
	if(dir.z < 0.0f)
		Swap(box.min.z, box.max.z);

	float rmin = Max( Max(
				(box.min.x - origin.x) * idir.x,
				(box.min.y - origin.y) * idir.y),
				(box.min.z - origin.z) * idir.z );
	float rmax = Min( Min(
				(box.max.x - origin.x) * idir.x,
				(box.max.y - origin.y) * idir.y),
				(box.max.z - origin.z) * idir.z );

	rmin = Max(0.0f, rmin);
	rmax = Max(0.0f, rmax);

	struct StackElem {
		int node;
		float min, max;
	} stack[64], *top = stack;

	int signs[3] = { idir.x < 0.0f, idir.y < 0.0f, idir.z < 0.0f };
	const Node *node = &nodes[startNode];

	while(true) {
		if(node->max < cmin || rmin >= rmax) {
			if(top == stack)
				return 0;

			top--;
			rmin = top->min;
			rmax = top->max;
			node = &nodes[top->node];
			continue;
		}
		if(node->IsLeaf() || node->min == node->max)
			return node->max;

		int axis = node->axis;
		int sign = signs[axis];
		float t = (node->split - (&origin.x)[axis]) * (&idir.x)[axis];

		if(rmax <= t) {
			node = &nodes[node->child[sign]];
			continue;
		}
		if(rmin >= t) {
			node = &nodes[node->child[sign ^ 1]];
			continue;
		}

		top->min = t;
		top->max = rmax;
		top->node = node->child[sign ^ 1];
		top++;

		rmax = t;
		node = &nodes[node->child[sign]];
	}
}


