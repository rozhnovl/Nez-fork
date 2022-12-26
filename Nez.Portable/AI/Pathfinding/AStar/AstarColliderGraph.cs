using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace Nez.AI.Pathfinding
{
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
		List<Vector2> _checkedPoints = new List<Vector2>(4);
		private float speed;
		private Vector2 goal;

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
		public List<Vector2> Search(Vector2 start, Vector2 goal)
		{
			var goalMotion = goal - start;
			var localGoal = goalMotion;
			int maxFitAttempts = 1000;
			while (SourceCollider.CollidesWithAny(ref goalMotion, out var cr) && maxFitAttempts-- > 0)
			{
				if (float.IsNaN(goalMotion.X) || float.IsNaN(goalMotion.Y))
					return new List<Vector2>();
				localGoal *= 0.95f;
				goal = start + localGoal;
				goalMotion = localGoal;
				this.goal = goal;
			}
			return AStarPathfinder.SearchIncludeGoalNeighbors(this, start, goal);
		}


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
			//_checkedPoints = new List<Vector2>();
			foreach (var dir in Dirs)
			{
				var dirCopy = dir;
				dirCopy.Normalize();
				dirCopy *= halfDistance;
				var next = node + dirCopy;
				if (!IsNodeInBounds(next))
					continue;
				if (_checkedPoints.Any(cp=>(cp-next).Length()<halfDistance/2))
					continue;
				_checkedPoints.Add(next);
				if (SourceCollider.CollidesWithAnyOnMove(node, ref dirCopy, out var cr))
				{
					//TODO wierd behavior when pused through two box colliders
					if (dirCopy.Length() < halfDistance / 2 || dirCopy.Length() > halfDistance)
						continue;
					else
					{
						dirCopy.Normalize();
						dirCopy *= halfDistance / 2;
						next = node + dirCopy;
					}
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