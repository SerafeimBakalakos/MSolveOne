using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace MGroup.Solvers.DDM.LagrangeMultipliers
{
	public class LagrangeMultiplier : IComparable<LagrangeMultiplier>
	{
		public LagrangeMultiplier(int nodeID, int dofID, int subdomainPlus, int subdomainMinus, int localIdx)
		{
			NodeID = nodeID;
			DofID = dofID;
			SubdomainMinus = subdomainMinus;
			SubdomainPlus = subdomainPlus;
			this.LocalIdx = localIdx;
		}

		public int DofID { get; }

		public int LocalIdx { get; }

		public int NodeID { get; }

		public int SubdomainMinus { get; }

		public int SubdomainPlus { get; }

		public int CompareTo(LagrangeMultiplier other)
		{
			if (other.NodeID - this.NodeID != 0)
			{
				return other.NodeID - this.NodeID;
			}
			else if (other.DofID - this.DofID != 0)
			{
				return other.DofID - this.DofID;
			}
			else
			{
				Debug.Assert((other.SubdomainPlus == this.SubdomainPlus) && (other.SubdomainMinus == this.SubdomainMinus));
				return 0;
			}
		}
	}
}
