using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.MSolve.Discretization
{
	public class ActiveDofs
	{
		private readonly Dictionary<int, IDofType> idsToDofs = new Dictionary<int, IDofType>();
		private readonly Dictionary<IDofType, int> dofsToIds = new Dictionary<IDofType, int>();
		private readonly object insertionLock = new object();
		private int nextId = 0;

		public void AddDof(IDofType dof)
		{
			lock (insertionLock)
			{
				bool exists = dofsToIds.ContainsKey(dof);
				if (!exists)
				{
					dofsToIds[dof] = nextId;
					idsToDofs[nextId] = dof;
					++nextId;
				}
			}
		}

		public void Clear()
		{
			lock (insertionLock)
			{
				idsToDofs.Clear();
				dofsToIds.Clear();
				nextId = 0;
			}
		}

		public int GetIdOfDof(IDofType dof) => dofsToIds[dof];

		public IDofType GetDofWithId(int id) => idsToDofs[id];

		public void RemoveDof(IDofType dof)
		{
			lock (insertionLock)
			{
				bool exists = dofsToIds.Remove(dof, out int id);
				if (exists)
				{
					idsToDofs.Remove(id);
				}
			}
		}
	}
}
