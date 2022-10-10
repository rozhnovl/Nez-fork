using System;
using Microsoft.Xna.Framework;
using System.Collections.Generic;
using System.Linq;
using Nez.Tiled;


namespace Nez.AI.Pathfinding
{
	/// <summary>
	/// basic static grid graph for use with A*. Add walls to the walls HashSet and weighted nodes to the weightedNodes HashSet. This provides
	/// a very simple grid graph for A* with just two weights: defaultWeight and weightedNodeWeight.
	/// </summary>
	public class AstarGridGraph : IAstarGraph<Point>
	{
		public List<Point> Dirs = new List<Point>
		{
			new Point(1, 0),
			new Point(0, -1),
			new Point(-1, 0),
			new Point(0, 1),

			new Point(1, 1),
			new Point(-1, -1),
			new Point(1, -1),
			new Point(-1, 1),
		};

		public HashSet<Point> Walls = new HashSet<Point>();
		public HashSet<Point> WeightedNodes = new HashSet<Point>();
		public int DefaultWeight = 1;
		public int WeightedNodeWeight = 5;

		int _width, _height;
		List<Point> _neighbors = new List<Point>(4);


		public AstarGridGraph(int width, int height)
		{
			_width = width;
			_height = height;
		}

		/// <summary>
		/// creates a WeightedGridGraph from a TiledTileLayer. Present tile are walls and empty tiles are passable.
		/// </summary>
		/// <param name="tiledLayer">Tiled layer.</param>
		public AstarGridGraph(TmxLayer tiledLayer)
		{
			_width = tiledLayer.Width;
			_height = tiledLayer.Height;

			for (var y = 0; y < tiledLayer.Map.Height; y++)
			{
				for (var x = 0; x < tiledLayer.Map.Width; x++)
				{
					if (tiledLayer.GetTile(x, y) != null)
						Walls.Add(new Point(x, y));
				}
			}
		}

		/// <summary>
		/// ensures the node is in the bounds of the grid graph
		/// </summary>
		/// <returns><c>true</c>, if node in bounds was ised, <c>false</c> otherwise.</returns>
		bool IsNodeInBounds(Point node)
		{
			return 0 <= node.X && node.X < _width && 0 <= node.Y && node.Y < _height;
		}

		/// <summary>
		/// checks if the node is passable. Walls are impassable.
		/// </summary>
		/// <returns><c>true</c>, if node passable was ised, <c>false</c> otherwise.</returns>
		bool IsNodePassable(Point node) => !Walls.Contains(node);

		/// <summary>
		/// convenience shortcut for calling AStarPathfinder.search
		/// </summary>
		public List<Point> Search(Point start, Point goal) => AStarPathfinder.Search(this, start, goal);

		#region IAstarGraph implementation

		IEnumerable<Point> IAstarGraph<Point>.GetNeighbors(Point node)
		{
			_neighbors.Clear();

			foreach (var dir in Dirs)
			{
				var next = new Point(node.X + dir.X, node.Y + dir.Y);
				if (IsNodeInBounds(next) && IsNodePassable(next))
					_neighbors.Add(next);
			}

			return _neighbors;
		}

		int IAstarGraph<Point>.Cost(Point from, Point to)
		{
			return WeightedNodes.Contains(to) ? WeightedNodeWeight : DefaultWeight;
		}

		int IAstarGraph<Point>.Heuristic(Point node, Point goal)
		{
			return Math.Abs(node.X - goal.X) + Math.Abs(node.Y - goal.Y);
		}

		#endregion
	}

	public class AstarColliderGraph : IAstarGraph<Vector2>
	{
		public List<Vector2> Dirs = new List<Vector2>
		{
			new Vector2(1, 0),
			new Vector2(0, -1),
			new Vector2(-1, 0),
			new Vector2(0, 1),

			new Vector2(1, 1),
			new Vector2(-1, -1),
			new Vector2(1, -1),
			new Vector2(-1, 1),
		};

		public Collider SourceCollider;

		int _width, _height;
		List<Vector2> _neighbors = new List<Vector2>(4);
		private float speed;
		private readonly Vector2 goal;

		public AstarColliderGraph(int width, int height, Collider collider, float speed, Vector2 goal)
		{
			_width = width;
			_height = height;
			SourceCollider = collider;
			this.speed = speed;
			this.goal = goal;
		}
		
		/// <summary>
		/// ensures the node is in the bounds of the grid graph
		/// </summary>
		/// <returns><c>true</c>, if node in bounds was ised, <c>false</c> otherwise.</returns>
		bool IsNodeInBounds(Vector2 node)
		{
			return 0 <= node.X && node.X < _width && 0 <= node.Y && node.Y < _height;
		}
		

		/// <summary>
		/// convenience shortcut for calling AStarPathfinder.search
		/// </summary>
		public List<Vector2> Search(Vector2 start, Vector2 goal) => AStarPathfinder.Search(this, start, goal);

		#region IAstarGraph implementation

		IEnumerable<Vector2> IAstarGraph<Vector2>.GetNeighbors(Vector2 node)
		{
			_neighbors.Clear();

			var halfDistance = (goal - node).Length()/2;
			halfDistance = Math.Min(halfDistance, SourceCollider.Bounds.Width);
			if (halfDistance < speed)
			{
				_neighbors.Add(goal);
				return _neighbors;
			}
			foreach (var dir in Dirs)
			{
				var dirCopy = dir;
				dirCopy.Normalize();
				dirCopy *= halfDistance;
				var next = node + dirCopy;
				if (!IsNodeInBounds(next))
					continue;
				
				if (SourceCollider.CollidesWithAnyOnMove(node, ref dirCopy, out var cr))
				{
					if (dirCopy.Length() < halfDistance / 2)
						continue;
					else
						next = node + dirCopy;
				}

				next.X = Mathf.Round(next.X);
				next.Y = Mathf.Round(next.Y);
				Debug.DrawLine(node, next, Color.Aqua);
				_neighbors.Add(next);
			}

			return _neighbors;
		}

		int IAstarGraph<Vector2>.Cost(Vector2 from, Vector2 to)
		{
			return (int)((from - to).Length());
		}

		int IAstarGraph<Vector2>.Heuristic(Vector2 node, Vector2 goal)
		{
			return (int)((goal - node).LengthSquared());
		}

		#endregion
	}
}