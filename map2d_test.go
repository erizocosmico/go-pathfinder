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
