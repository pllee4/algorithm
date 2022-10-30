## Variants of A*

### Equation

f = w1 * g + w2 * h

| w   | g   | Algorithm                                                                            |
| --- | --- | ------------------------------------------------------------------------------------ |
| 1   | 0   | Dijkstra's Algorithm. This is traditional graph search without a heuristic.          |
| 0   | 1   | Pure Heuristic Search. This is also known as greedy best-first search.               |
| 1   | 1   | This is the regular A* algorithm.                                                    |
| 1   | 10  | Weighted A*. Weighted A* gives up optimality to try to find a solution more quickly. |
