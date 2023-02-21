import random
from math import ceil
from queue import PriorityQueue


class Node(object):
	def __init__(self, label: str=None):
		self.label = label
		self.children = []
		
	def __lt__(self,other):
		return (self.label < other.label)
	
	def __gt__(self,other):
		return (self.label > other.label)
	
	def __repr__(self):
		return 'Node({})'.format(self.label)
	
	def add_child(self, node, cost=1):
		edge = Edge(self, node, cost)
		self.children.append(edge)
	
	
class Edge(object):
	def __init__(self, source: Node, destination: Node, cost: int=1):
		self.source = source
		self.destination = destination
		self.cost = cost
	
	def __repr__(self):
		return '{}: {}'.format(self.destination.label, self.cost)

def GenerateGrid(n, p):
	'''
	Generates undirected mesh grid with n rows and n columns
	'''
	g, c = [[] for _ in range(n*n)], {}
	for node in range(n*n):
		try:
			col = ([_ for _ in range(n*ceil((node+1)/n)-n, n*ceil((node+1)/n))])
			adj_nodes = [i for i  in col if i != node and abs(node-i)==1]
			adj_nodes += [i for i in [node+n, node-n] if i>=0 and i<n*n]
			for an in adj_nodes:
				if an >= 0 and an < n*n:
					g[node].append(an)
					if (an, node) not in c:
						c[(node, an)] = random.randint(1,20)
					else:
						c[(node, an)] = c[(an, node)]            
		except IndexError:
			pass
	
	for _ in range(int(p/(2*n*(n-1)))):
		s = random.choice([_ for _ in range(0,n*n)])
		t = random.choice(g[s])
		print(s, t)
		g[s].remove(t)
		g[t].remove(s)
		del	c[(s,t)]
		del c[(t,s)]

	_g = []
	for _l in range(n*n):
		_g.append(Node(_l))
	for (_s,_t), _c in c.items():
		_g[_s].add_child(_g[_t], _c)

	return _g

def UniformCostSearch(s, t):
	q = PriorityQueue()
	q.put((0, [s]))
	while not q.empty():
		p = q.get()
		cur = p[1][-1]
		if cur.label == t:
			return p[1]
		for e in cur.children:
			np = list(p[1])
			np.append(e.destination)
			q.put((p[0] + e.cost, np))

def IterativeDeepeningDepthFirstSearch(_source: Node, _target: str, _limit: int=10):
	def _dls(p: list, _t: str, _l: int):
		cur = p[-1]
		if cur.label == _t:
			return p
		if _l <= 0:
			return None
		for e in cur.children:
			np = list(p)
			np.append(e.destination)
			r = _dls(np, _t, _l - 1)
			if r is not None:
				return r
	
	for depth in range(0, _limit):
		r = _dls([_source], _target, depth)
		if r is None:
			continue
		return r
	
	print('goal not in graph with depth {}'.format(_limit))
	return None


def AstarSearch(_source : Node, _target : Node, _graph : list, n: int):
	def generateHeuristicValue(_s : Node, _t : Node, _n : int):
		t = {}
		tx = ceil((int(_t.label)+1)/_n)
		ty = int(_t.label)-(tx-1)*_n+1
		for _node in _graph:
			nx = ceil((int(_node.label)+1)/_n)
			ny = int(_node.label)-(nx-1)*_n+1
			t[_node] = abs(tx-nx)+abs(ty-ny)
		return t
	h : dict = generateHeuristicValue(_source, _target, n)
	#print(f"{_source.label},{_target.label} - {h}")

	open = set([_source])
	close = set()
	g = {}
	g[_source] = 0
	parents = {}
	parents[_source] = _source

	while len(open):
		_node = None
		#print(f"g={g} and h={h}")
		for v in open:
			if _node == None or g[v]+h[v] < g[_node]+h[_node]:
				_node = v
		if _node == None:
			print("Path does not exist")
			return None
		if _node == _target:
			reconst_path = []
			while parents[_node] != _node:
				reconst_path.append(_node)
				_node = parents[_node]
			reconst_path.append(_source)

			reconst_path.reverse()

			#print('Path found: {}'.format(reconst_path))
			return reconst_path
		
		for _neighbor in _node.children:
			if _neighbor.destination not in open and _neighbor.destination not in close:
				open.add(_neighbor.destination)
				parents[_neighbor.destination] = _node
				g[_neighbor.destination] = g[_node] + _neighbor.cost
			else:
				if g[_neighbor.destination] > g[_node]+_neighbor.cost:
					g[_neighbor.destination] = g[_node] + _neighbor.cost
					parents[_neighbor.destination] = _node

					if _neighbor.destination in close:
						close.remove(_neighbor.destination)
						open.add(_neighbor.destination)
		open.remove(_node)
		close.add(_node)
	print('Path does not exist!')
	return None

if __name__=='__main__':
	n = int(input("Enter the number n of rows and columns of the grid: "))
	p = int(input("Enter the percentage p of edges to be removed: "))
	g = GenerateGrid(n, p)
	s : Node = random.choice(g)
	t : Node = random.choice([_ for _ in g if _ != s])

	p_ucs_path = ""
	p_ucs_cost = 0
	ucs_path = ((UniformCostSearch(s, t.label)))
	for i in range(len(ucs_path)):
		if i==len(ucs_path)-1:
			p_ucs_path+=str(ucs_path[i])
		else:
			p_ucs_path+=str(ucs_path[i])+"->"
			for child in ucs_path[i].children:
				if child.destination == ucs_path[i+1]:
					p_ucs_cost+=child.cost
	print(f"For Uniform Cost Search Search: \n\tPath is {p_ucs_path} \n\tCost is {p_ucs_cost} \n\tNumber of nodes created is 0")

	p_iddfs_path = ""
	p_iddfs_cost = 0
	iddfs_path = (IterativeDeepeningDepthFirstSearch(s, t.label, n*n))
	if iddfs_path:
		for i in range(len(iddfs_path)):
			if i==len(iddfs_path)-1:
				p_iddfs_path+=str(iddfs_path[i])
			else:
				p_iddfs_path+=str(iddfs_path[i])+"->"
				for child in iddfs_path[i].children:
					if child.destination == iddfs_path[i+1]:
						p_iddfs_cost+=child.cost
	print(f"For Iterative Deepening Depth-First Search: \n\tPath is {p_iddfs_path} \n\tCost is {p_iddfs_cost} \n\tNumber of nodes created is 0")

	p_as_path = ""
	p_as_cost = 0
	as_path = (AstarSearch(s,t,g,n))
	for i in range(len(as_path)):
		if i==len(as_path)-1:
			p_as_path+=str(as_path[i])
		else:
			p_as_path+=str(as_path[i])+"->"
			for child in as_path[i].children:
				if child.destination == as_path[i+1]:
					p_as_cost+=child.cost
	print(f"For A* Search Search: \n\tPath is {p_as_path} \n\tCost is {p_as_cost} \n\tNumber of nodes created is 0")