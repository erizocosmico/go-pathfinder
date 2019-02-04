package pathfinder

import (
	"errors"
	"math"
)

// Point in a 2D space.
type Point struct {
	// X coordinate.
	X uint64
	// Y coordinate.
	Y uint64
}

// NewPoint creates a point with the given X and Y coordinates.
func NewPoint(x, y uint64) Point {
	return Point{x, y}
}

// Equal returns whether the point is equal to the given point.
func (p Point) Equal(other Point) bool {
	return p.X == other.X && p.Y == other.Y
}

// Distance returns the distance between the point and the given one.
func (p Point) Distance(other Point) float64 {
	return math.Pow(float64(other.X)-float64(p.X), 2) +
		math.Pow(float64(other.Y)-float64(p.Y), 2)
}

// Contains implements the Shape interface.
func (p Point) Contains(other Point) bool {
	return p.Equal(other)
}

// Shape in a 2D space.
type Shape interface {
	// Contains returns whether the point is contained in the shape.
	Contains(Point) bool
}

type square struct {
	origin Point
	size   uint64
}

// Square returns a square shape given an origin point and a square side size.
func Square(origin Point, size uint64) Shape {
	return &square{origin, size}
}

func (s *square) Contains(p Point) bool {
	x, y := s.origin.X, s.origin.Y
	maxx, maxy := s.origin.X+s.size, s.origin.Y+s.size
	return p.X >= x && p.X <= maxx && p.Y >= y && p.Y <= maxy
}

// Map is a 2D map.
type Map struct {
	cache        map[Point]bool
	obstacles    []Shape
	xsize, ysize uint64
}

// NewMap creates a map with the given width and height.
func NewMap(xsize, ysize uint64) *Map {
	return &Map{
		cache: make(map[Point]bool),
		xsize: xsize,
		ysize: ysize,
	}
}

// AddObstacle adds as an obstacle the points of the given shape.
func (m *Map) AddObstacle(o Shape) {
	m.obstacles = append(m.obstacles, o)
}

// IsObstacle returns whether the given point is part of an obstacle.
func (m *Map) IsObstacle(p Point) bool {
	if is, ok := m.cache[p]; ok {
		return is
	}

	var occupied bool
	for _, o := range m.obstacles {
		if o.Contains(p) {
			occupied = true
			break
		}
	}

	m.cache[p] = occupied
	return occupied
}

// ErrCantReachGoal is returned when the goal could not be found.
var ErrCantReachGoal = errors.New("astar: could not reach goal")

// Neighbours returns all neighbours within a distance of 1. If diagonalMoves
// is true, it will also include the diagonal neighbours.
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

		if m.IsObstacle(newp) {
			continue
		}

		result = append(result, newp)
	}

	return result
}

// Path returns all the points that need to be visited in order to get from start
// to goal in the least amount of steps. If diagonalMoves is true, diagonal moves
// from a point to another will be allowed.
func (m *Map) Path(start, goal Point, diagonalMoves bool) ([]Point, error) {
	closedList := make(map[Point]struct{})
	openList := map[Point]struct{}{start: {}}
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
