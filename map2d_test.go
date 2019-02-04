package pathfinder

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestPath(t *testing.T) {
	m := NewMap(6, 6)

	// ......
	// ..###.
	// ..#...
	// ..###.
	// ......
	obstacles := []Point{
		NewPoint(2, 2),
		NewPoint(3, 2),
		NewPoint(4, 2),
		NewPoint(2, 3),
		NewPoint(2, 4),
		NewPoint(3, 4),
		NewPoint(4, 4),
	}

	for _, o := range obstacles {
		m.AddObstacle(o)
	}

	t.Run("diagonal moves", func(t *testing.T) {
		path, err := m.Path(NewPoint(1, 1), NewPoint(4, 3), true)
		require.NoError(t, err)

		expected := []Point{
			NewPoint(1, 1),
			NewPoint(2, 1),
			NewPoint(3, 1),
			NewPoint(4, 1),
			NewPoint(5, 2),
			NewPoint(4, 3),
		}

		require.Equal(t, expected, path)
	})

	t.Run("non-diagonal moves", func(t *testing.T) {
		path, err := m.Path(NewPoint(1, 1), NewPoint(4, 3), false)
		require.NoError(t, err)

		expected := []Point{
			NewPoint(1, 1),
			NewPoint(2, 1),
			NewPoint(3, 1),
			NewPoint(4, 1),
			NewPoint(5, 1),
			NewPoint(5, 2),
			NewPoint(5, 3),
			NewPoint(4, 3),
		}

		require.Equal(t, expected, path)
	})
}

func TestSquare(t *testing.T) {
	require := require.New(t)
	s := Square(NewPoint(1, 1), 1)

	require.True(s.Contains(NewPoint(1, 1)))
	require.True(s.Contains(NewPoint(1, 2)))
	require.True(s.Contains(NewPoint(2, 1)))
	require.True(s.Contains(NewPoint(2, 2)))
	require.False(s.Contains(NewPoint(0, 1)))
	require.False(s.Contains(NewPoint(3, 1)))
	require.False(s.Contains(NewPoint(1, 3)))
}

func TestPathCantReachGoal(t *testing.T) {
	m := NewMap(6, 6)

	// ......
	// ......
	// ..###.
	// ..#.#.
	// ..###.
	// ......
	obstacles := []Point{
		NewPoint(2, 2),
		NewPoint(3, 2),
		NewPoint(4, 2),
		NewPoint(2, 3),
		NewPoint(2, 4),
		NewPoint(3, 4),
		NewPoint(4, 4),
		NewPoint(4, 3),
	}

	for _, o := range obstacles {
		m.AddObstacle(o)
	}

	_, err := m.Path(NewPoint(0, 0), NewPoint(3, 3), true)
	require.Equal(t, ErrCantReachGoal, err)
}
