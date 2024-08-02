# Competitive-Programming-Cheat-Sheet

## 1. Sorting
- Sorting custom object which ```std::sort``` requires **<** operator to be defined by the class of that object, but keep in mind while defining **<** operator it follows weak comparison i.e if a < b and b < c then a < c.
 ```
 class Custom {
   public:
      bool operator<(const Custom& other) const {
          // do weak comparison here and return whether this object is less than the other or not
      }
  }
 ```


## 2. Binary Search
- Applying binary search over input space need not to be sorted in any order, if the input space specifies whether the target is present or not in constant time then search space can be squeezed to half thus applying binary search will be a feasible move.
  
- If result of optimization stratgy is linear i.e if finding some optimal solution guarantees that the lower optimal solution must exist, in that case you can use binary search over all optimal solution search space *if and only if finding that optimal solution over the input space is possible or not in linear or constant time complexity*.
 ```
  auto left = search_space.begin(), right = search_space.end();
  while (left <= right) {
    auto mid = left + std::distance(left, right) / 2;
    // function possible return bool, whether mid can be optimal solution or not
    if (possible(input_space.begin(), input_space.end(), mid) left = ++mid;
    else right = --mid;
  }
  return left;
 ```

- Any dynamic programming problem that involves finding optimal solution can be solved using binary search, it cannot be the optmized solution but it still be faster than the brute force. For example finding the longest common substring can be solved in O(n * m) * log (m) time which is slower than O(n * m).

- ```std::lower_bound(input_space.begin(), input_space.end(), target)``` returns the iterator pointing to the first element which is greater than or equal to the target if the input space is sorted by defining < operator while ```std::upper_bound(input_space.begin(), input_space.end(), target)``` returns the first element greater than the target. These two functions also take the optional custom lambda fucntion for comparision.

- ```std::set``` and ```std::multiset``` also has ```lower_bound``` and ```upper_bound``` methods defined that can be used when the input space is mutable and changing over time.
