# go-pathfinder [![GoDoc](https://godoc.org/github.com/erizocosmico/go-pathfinder?status.svg)](https://godoc.org/github.com/erizocosmico/go-pathfinder) [![Build Status](https://travis-ci.org/erizocosmico/go-pathfinder.svg?branch=master)](https://travis-ci.org/erizocosmico/go-pathfinder) [![codecov](https://codecov.io/gh/erizocosmico/go-pathfinder/branch/master/graph/badge.svg)](https://codecov.io/gh/erizocosmico/go-pathfinder) [![Go Report Card](https://goreportcard.com/badge/github.com/erizocosmico/go-pathfinder)](https://goreportcard.com/report/github.com/erizocosmico/go-pathfinder) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Pathfinder on 2D maps with obstacles using A* algorithm.

### Install

```
go get github.com/erizocosmico/go-pathfinder
```

### Usage

```go
// New square map of side 10.
m := pathfinder.NewMap(10, 10)

// Add an obstacle to the map, a square of side 3 with origin at (2, 2).
m.AddObstacle(pathfinder.Square(pathfinder.NewPoint(2, 2), 3))

start := pathfinder.NewPoint(1, 0)
end := pathfinder.NewPoint(9, 9)

// Find the path from (1, 0) to (9, 9) allowing diagonal moves.
path, err := m.Path(start, end, true)
// handle path and err

// Find the path from (1, 0) to (9, 9) NOT allowing diagonal moves.
path, err := m.Path(start, end, false)
// handle path and err
```

## License

MIT, see [LICENSE](/LICENSE)