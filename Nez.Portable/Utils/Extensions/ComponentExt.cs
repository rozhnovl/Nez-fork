using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Nez.AI.GOAP;


namespace Nez
{
	public static class ComponentExt
	{
		#region Entity Component management

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static T AddComponent<T>(this Component self, T component, System.Action<T> postAddAction = null) where T : Component
		{
			var c= self.Entity.AddComponent(component);
			postAddAction?.Invoke(c);
			return c;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static T AddComponent<T>(this Component self) where T : Component, new()
		{
			return self.Entity.AddComponent<T>();
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static T GetComponent<T>(this Component self) where T : Component
		{
			return self.Entity.GetComponent<T>();
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool HasComponent<T>(this Component self) where T : Component => self.Entity.HasComponent<T>();

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void GetComponents<T>(this Component self, List<T> componentList) where T : class
		{
			self.Entity.GetComponents<T>(componentList);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static List<T> GetComponents<T>(this Component self) where T : Component
		{
			return self.Entity.GetComponents<T>();
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool RemoveComponent<T>(this Component self) where T : Component
		{
			return self.Entity.RemoveComponent<T>();
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void RemoveComponent(this Component self, Component component)
		{
			self.Entity.RemoveComponent(component);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void RemoveComponent(this Component self)
		{
			self.Entity.RemoveComponent(self);
		}

		#endregion
	}
}