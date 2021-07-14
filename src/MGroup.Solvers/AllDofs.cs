using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

//TODO: IDofType should provide an ID like INode. Otherwise I should use enums for dof types and XFEM should find a different way
//		for its enriched dofs.
namespace MGroup.Solvers
{
	public static class AllDofs
	{
		private static readonly Dictionary<int, IDofType> idsToDofs = new Dictionary<int, IDofType>();
		private static readonly Dictionary<IDofType, int> dofsToIds = new Dictionary<IDofType, int>();
		private static readonly object insertionLock = new object();
		private static int nextCode = 0;

		public static void AddDof(IDofType dof)
		{
			lock (insertionLock)
			{
				bool exists = dofsToIds.ContainsKey(dof);
				if (!exists)
				{
					dofsToIds[dof] = nextCode;
					idsToDofs[nextCode] = dof;
					++nextCode;
				}
			}
		}

		//public static void AddStructuralDofs()
		//{
		//	AddDof(StructuralDof.TranslationX);
		//	AddDof(StructuralDof.TranslationY);
		//	AddDof(StructuralDof.TranslationZ);
		//	AddDof(StructuralDof.RotationX);
		//	AddDof(StructuralDof.RotationY);
		//	AddDof(StructuralDof.RotationZ);
		//}

		public static void Clear()
		{
			lock (insertionLock)
			{
				idsToDofs.Clear();
				dofsToIds.Clear();
				nextCode = 0;
			}
		}

		public static int GetIdOfDof(IDofType dof) => dofsToIds[dof];

		public static IDofType GetDofWithId(int id) => idsToDofs[id];
	}
}
