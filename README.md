# Competitive Programming Cheat Sheet

## 0. Invariant
- An invariant of an algorithm is a condition that remains true throughout the execution of the algorithm, typically within a loop or recursive call. Invariants are crucial in ensuring that an algorithm works correctly and terminates as expected.

- When designing any algorithm first define the invariants and check after every step in the execution flow, does invariants hold true, if it does then that algorithm is correct, robust and must stop in finite steps.
```cpp
// For example lets see the invariant of binary search, find whether
// an element x is present in the
// sorted array of integers a[0...n - 1]
int l = 0, r = n - 1;
// Invariant: if the given element is present in the array,
//            it must be in between index l and r
//            i.e. each step in the execution flow a[l] < x && a[r] > x.
//            If the given element is present in the
//            array then l == r hold true in the end.
while (l <= r) {
   int m = l + (r - l) / 2;
   if (a[m] == x) return true;
   if (a[m] < x) l = m + 1;  // Here invairant holds true as the array is sorted and
                             // if the target element is present,
                             // then it should be in the other half of the range [l, r]
   else r = m - 1;
}
return false;
```

- For example, in Dijkstra's algorithm, the main invariant is that once a node's shortest path estimate is marked as "final" (i.e., the node is added to the set of processed nodes), this estimate is indeed the shortest possible path from the source to that node. This invariant ensures the correctness of the algorithm. 

## 1. Sorting
- Sorting custom object which ```std::sort``` requires **<** operator to be defined by the class of that object, but keep in mind while defining **<** operator it follows weak comparison i.e if a < b and b < c then a < c.
 ```cpp
 class Custom {
   public:
      bool operator<(const Custom& other) const {
          // do weak comparison here and return whether
          // this object is less than the other or not
      }
  }
 ```
- ```std::stable_sort``` can be used if the order of input iterator is to be maintained while sorting other wise custom comparator can be used to do so.
- In quick sort, use two pointer to store all the elements greater than the pivot element while iterating over the array,
```cpp
   // keep left pointer to the end of sequence less the pivot and
   // right pointer to the start of the sequence which is greater or equal to pivot
   if (arr[right] < pivot) {
      left++;
      swap(arr[left], arr[right]);
   }
   right++;
```

- Interval Scheduling Maximization algorithm is used to select the maximum number of non-overlapping intervals (or activities) from a given set of intervals.The idea is to always pick the interval that ends the earliest (and doesn't overlap with previously selected intervals). This strategy ensures that you leave the most room for future intervals to be included. 


## 2. Binary Search
- Designing binary search solution can be tricky as one can make mistake in defining the invariants or somes step does not holds true for the defined invaraints during the execution  of the algorithm.
```cpp
// Find the index of the given element x in the sorted array a[0....n - 1] if it exists.
int l = 0, r = n - 1;
// Invariant: l -> points to the last element which is less than x.
              r -> points to the first element which is greater than x.
              In the end return if the target is present the l and r converge to a single index
while (l <= r) {
   int m = l + (r - l) / 2;
   if (a[m] == x) return m;
   else if (a[m] < x) l = m + 1; // Note that m is always in the range [0, n -1]
                                 // as m is calculated using integer division
   else r = m - 1;
}
return -1;
```
   
- Binary search can be applied even if the input space is unsorted, as long as the search space allows us to determine in constant time whether the target is present in a specific range. By iteratively halving the search space based on this condition, binary search becomes a feasible and efficient approach.
  
- If the result of an optimization strategy is linear, meaning that finding a higher optimal solution ensures the existence of all lower optimal solutions, then binary search can be applied over the entire range of optimal solutions—*provided that determining whether a given solution is optimal can be done in linear or constant time*.
  
 ```cpp
  auto left = min_possible_search, right = max_possible_search;
  // Invariant:  left -> points to last solution which is possible
  //              right -> points to first solution which is not possible
  while (right - left > 1) {
    auto mid = l + (r - l) / 2;
    // function possible return bool, whether mid can be optimal solution or not
    if (possible(input_space.begin(), input_space.end(), mid) left = mid;
    else right = mid;
  }
  return left;
 ```

- Binary search can be used to find the last or the nth possible solution within the search space if it follows 1 1 1 1 1 1 0 0 0 0 0 0 0  pattern in serach space. (1 means possible and 0 means not possible). For example, this pattern arises in finding the kth smallest or larget element in the sequence where finding the complete sequence is not feasible. In that case, define **left** as the number of elements which are smaller than or equal to **K** and **right** as the number of elements which are greater than **k** and perform binary search over **left** and **right**, then in the end **left** will point to kth smallest element.

- Any dynamic programming problem that involves finding optimal solution can be solved using binary search, it cannot be the optmized solution but it still be faster than the brute force. For example finding the longest common substring can be solved in O(n * m) * log (m) time which is slower than O(n * m).

- ```std::lower_bound(input_space.begin(), input_space.end(), target)``` returns the iterator pointing to the first element which is greater than or equal to the target if the input space is sorted by defining < operator while ```std::upper_bound(input_space.begin(), input_space.end(), target)``` returns the first element greater than the target. These two functions also take the optional custom lambda fucntion for comparision.

- ```std::set```, ```std::map``` and ```std::multiset``` also has ```lower_bound``` and ```upper_bound``` methods defined that can be used when the input space is mutable and changing over time.

```cpp
// This is the stl implementation of std::lower_bound...
 _DistanceType __len = std::distance(__first, __last);
while (__len > 0)
{
	  _DistanceType __half = __len >> 1;
	  _ForwardIterator __middle = __first;
	  std::advance(__middle, __half);
	  if (__comp(__middle, __val))  // While defining custom comparator for std::lower_bound,
                                        // follow this comparator function
	    {                           // defnition. It should return true __middle < __val
	      __first = __middle;
	      ++__first;
	      __len = __len - __half - 1;
	    }
	  else
	    __len = __half;
}
return __first;

// Define comparator function like this
bool operator<(const Custom& obj, T val) const {
   // return true if obj is less than val
}
```

- To find lower_bound and upper_bound of an element in the given input space, use this
```cpp
... // a sorted array is stored as a[0], a[1], ..., a[n-1]
int lb = -1, ub = n;
// Invariant: lb -> points to the first element which is greater than or equal to the target x
//            ub -> points to the first element which is greater than the target x
while (ub - lb > 1) {
    int m = lb + (ub - lb) / 2;
    if (a[m] <= x) lb = m;
    else ub = m;
}
return {lb, ub};
```

## 3. String
- Computing hash of string can be used to determine whether two string are equal or not in O(n) time with the probability that collision happens is only $\approx \frac{1}{m}$ . For $m = 10^9 + 9$  the probability is $\approx 10^{-9}$  which is quite low.
```cpp
long long compute_hash(const std::string& s) {
    const int p = 31;
    const int m = 1e9 + 9;
    long long hash_value = 0;
    long long p_pow = 1;
    for (char c : s) {
        hash_value = (hash_value + (c - 'a' + 1) * p_pow) % m;
        p_pow = (p_pow * p) % m;
    }
    return hash_value;
}
```

- Finding longest prefix which is also the suffix at every index in the pattern, is used in KMP string matching algorithm, which is computed like this,
```cpp
 vector<int> prefix_function(const std::string& s) {
    std::vector<int> pi(s.size());
    for (int i = 1; i < s.size(); i++) {
        int j = pi[i-1];
        while (j > 0 && s[i] != s[j]) j = pi[j-1];
        if (s[i] == s[j]) j++;
        pi[i] = j;
    }
    return pi;
}
```

## 3. Graph Theory
- Finding the shortest path from a start node to an end node is a fundamental problem in graph theory. Different graph structures require different algorithms to efficiently determine the shortest path.

- Case 1: Trees
A tree is a special type of graph with no cycles and exactly one path between any two nodes. In a tree, both BFS (Breadth-First Search) and DFS (Depth-First Search) can be used to find the shortest path between the start and end nodes. This is because, in a tree:

The first time a node is discovered during a traversal (either BFS or DFS), it is reached via the shortest path from the start node.
BFS is generally preferred because it explicitly explores all nodes at the present depth level before moving on to nodes at the next depth level, ensuring that the first time a node is reached, it is by the shortest path. However, in the context of a tree, DFS will also work since there are no cycles, and the first encounter of a node via DFS is along the shortest path as well.

```cpp
   int bfs(int start, int end, const std::vector<std::vector<std::pair<int, int>>>& tree) {
          std::queue<int> q;
          std::vector<int> dis(tree.size());
          std::vector<bool> vis(tree.size());
          q.push(start);
          dis[start] = 0;
          vis[start] = true;
          while (!q.empty()) {
                auto u = q.front();
                q.pop();
                for (const auto& v : tree[u]) {
                    if (!vis[v.first]) {
                       dis[v.first] = dis[u] + v.second;
                       vis[v.first] = true;
                    }
                } 
          }
          return dis[end];
   }
```

- Case 2: General Graphs (with cycles)
In a general graph, especially one that may contain cycles, BFS is typically used to find the shortest path in an unweighted graph:

BFS ensures that the first time a node is reached, it is by the shortest path, as it explores all neighbors of a node before moving on to the next level.
DFS does not guarantee finding the shortest path in graphs with cycles, as it may explore a longer path before backtracking.

```cpp
    int bfs(int start, int end, const std::vector<std::vector<int>>& graph) {
          std::queue<int> q;
	  std::vector<int> dis(graph.size());
	  std::vector<bool> vis(graph.size());
	  q.push(start);
          int level = 0;
          vis[start] = true;
          dis[start] = 0;
	  while (!q.empty()) {
		int sz = q.size();
                level++;
                // do a level order traversal
                while (sz--) {
                     auto u = q.front();
                     for (const auto& v : graph[u]) {
                         if (!vis[v]) {
                            q.push(v);
                            vis[v] = true; // mark the node visited when the first time
                                           // it is discovered
                            dis[v] = leve;
                         }
                     }
                }
	  }
	  return dis[end]; 
    }
```

- Case 3: Weighted Graphs
For graphs with weighted edges, neither simple BFS nor DFS will find the shortest path efficiently. Instead, algorithms like Dijkstra's Algorithm or Bellman-Ford Algorithm are used:

Dijkstra's Algorithm is efficient for graphs with non-negative edge weights and finds the shortest path from the start node to all other nodes.
Bellman-Ford Algorithm can handle graphs with negative edge weights and will also detect if a negative-weight cycle exists.

```cpp
	void dijkstra(int start, int end,
                      const std::vector<std::vector<std::pair<int, int>>& graph) {
	    std::set<std::pair<int, int>> q;
            std::vector<int> dis(graph.size(), INT_MAX);
	    q.insert({0, s});
	    while (!q.empty()) {
	        int from = q.begin()->second;
	        q.erase(q.begin());
	        for (cosnt auto& edge : graph[from]) {
	            int to = edge.first;
	            int len = edge.second;
	            if (dis[from] + len < d[to]) {
	                q.erase({dis[to], to});
	                d[to] = dis[from] + len;
	                q.insert({dis[to], to});
	            }
	        }
	    }
            return dis[end];
	}

    // The trick Dijkstra's Algorithm uses to find the shortest path is when
    // it releases a node from the set of active nodes. It greadly chooses
    // the node with shortest calculated distance as it correctly update the
    // distance of the node which are already present in the set of active nodes,
    // lets see

   //      0
   //    /   \
   //   2     6
   //  /       \
   // 1---3-----2

  // In the above scenerio, at one point in the executiion of algorithm
  // set of active nodes = {1, 2} with distance 2 and 6 respenctively, if you
  // chose node 2 to eliminate first then it calculated distance will be 2 and 6
  // respectively which is wrong, it shoulde 2 and 5  
```

- When finding the path between two nodes during **BFS** or **DFS** traversal, maintaining a **parent** array is often more efficient and intuitive than relying on a **stack** to reconstruct the path. The **parent** array directly stores the predecessor of each node, making it straightforward to trace the path from the destination node back to the start node.

- opological sorting is a linear ordering of vertices in a directed acyclic graph (DAG) such that for every directed edge u → v, vertex  𝑢 comes before vertex 𝑣 in the ordering. This concept is particularly useful in scenarios like task scheduling, where certain tasks must be performed before others.

```cpp
void topologicalSortDFS(int v, std::vector<bool>& visited, std::stack<int>& st,
                               const std::vector<std::vector<int>>& adj) {
    visited[v] = true;
    for (int i : adj[v]) {
        if (!visited[i]) topologicalSortDFS(i, visited, Stack, adj);
    }
    st.push(v); // Add the current node after visiting of its descendents
}


void topologicalSortBFS(int V, std::vector<std::vector<int>> &adj) {
    std::vector<int> in_degree(V, 0);
    // Calculate in-degree of each vertex
    for (int u = 0; u < V; u++) {
        for (int v : adj[u]) {
            in_degree[v]++;
        }
    }
    // Create a queue to store vertices with in-degree 0
    std::queue<int> q;
    for (int i = 0; i < V; i++) {
        if (in_degree[i] == 0) {
            q.push(i);
        }
    }
    // Initialize a counter to count visited vertices
    int count = 0;
    // Vector to store the result (i.e., topological order)
    std::vector<int> top_order;
    // Process vertices in the queue
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        top_order.push_back(u);
        // Decrease in-degree of all adjacent vertices
        for (int v : adj[u]) {
            in_degree[v]--;
            // If in-degree becomes 0, add it to the queue
            if (in_degree[v] == 0) {
                q.push(v);
            }
        }
        count++;
    }

    // Check if there was a cycle
    if (count != V) {
        std::cout << "There exists a cycle in the graph\n";
    } else {
        // Print the topological order
        for (int i : top_order) {
            std::cout << i << " ";
        }
        std::cout << "\n";
    }
}
```
