package pathfinder

import (
	"errors"
	"math"
)

type Point struct {
	X uint64
	Y uint64
}

func NewPoint(x, y uint64) Point {
	return Point{x, y}
}

func (p Point) Equal(other Point) bool {
	return p.X == other.X && p.Y == other.Y
}

func (p Point) Distance(other Point) float64 {
	return math.Pow(float64(other.X)-float64(p.X), 2) +
		math.Pow(float64(other.Y)-float64(p.Y), 2)
}

func (p Point) Points() []Point {
	return []Point{p}
}

type Shape interface {
	Points() []Point
}

type square struct {
	origin Point
	size   uint64
}

func Square(origin Point, size uint64) Shape {
	return &square{origin, size}
}

func (s *square) Points() []Point {
	var result = make([]Point, s.size*s.size)
	for i := uint64(0); i < s.size; i++ {
		for j := uint64(0); j < s.size; j++ {
			result[i*s.size+j] = NewPoint(s.origin.X+j, s.origin.Y+i)
		}
	}
	return result
}

type Map struct {
	obstacles    map[Point]struct{}
	xsize, ysize uint64
}

func NewMap(xsize, ysize uint64) *Map {
	return &Map{
		obstacles: make(map[Point]struct{}),
		xsize:     xsize,
		ysize:     ysize,
	}
}

func (m *Map) AddObstacle(o Shape) {
	for _, p := range o.Points() {
		m.obstacles[p] = struct{}{}
	}
}

var ErrCantReachGoal = errors.New("astar: could not reach goal")

func (m *Map) Neighbours(p Point, diagonalMoves bool) []Point {
	var result []Point

	var ne = neighbours
	if diagonalMoves {
		ne = diagNeighbours
	}

	for _, n := range ne {
		if p.X == 0 && n[0] < 0 ||
			p.Y == 0 && n[1] < 0 {
			continue
		}

		newx := uint64(int64(p.X) + n[0])
		newy := uint64(int64(p.Y) + n[1])

		if newx >= m.xsize || newy >= m.ysize {
			continue
		}

		newp := NewPoint(newx, newy)

		if _, ok := m.obstacles[newp]; ok {
			continue
		}

		result = append(result, newp)
	}

	return result
}

func (m *Map) Path(start, goal Point, diagonalMoves bool) ([]Point, error) {
	closedList := make(map[Point]struct{})
	openList := map[Point]struct{}{start: struct{}{}}
	fscore := map[Point]float64{start: 0}
	gscore := map[Point]float64{start: 0}
	cameFrom := make(map[Point]Point)

	for len(openList) > 0 {
		var current Point
		var currentScore = math.MaxFloat64

		for node := range openList {
			score := fscore[node]
			if score < currentScore {
				current = node
				currentScore = score
			}
		}

		delete(openList, current)
		closedList[current] = struct{}{}

		if current.Equal(goal) {
			var path []Point

			for {
				path = append(path, current)
				var ok bool
				current, ok = cameFrom[current]
				if !ok {
					break
				}
			}

			var revPath = make([]Point, len(path))
			for i, node := range path {
				revPath[len(path)-1-i] = node
			}

			return revPath, nil
		}

	N:
		for _, n := range m.Neighbours(current, diagonalMoves) {
			if _, ok := closedList[n]; ok {
				continue
			}

			gscore[n] = gscore[current] + 1
			fscore[n] = float64(gscore[n]) + n.Distance(goal)

			for on := range openList {
				if n.Equal(on) {
					if g, ok := gscore[on]; ok && gscore[n] > g {
						continue N
					}
				}
			}

			openList[n] = struct{}{}
			cameFrom[n] = current
		}
	}

	return nil, ErrCantReachGoal
}

var diagNeighbours = [][2]int64{
	{-1, -1},
	{0, -1},
	{1, -1},
	{-1, 0},
	{1, 0},
	{-1, 1},
	{0, 1},
	{1, 1},
}

var neighbours = [][2]int64{
	{0, -1},
	{-1, 0},
	{0, 1},
	{1, 0},
}
