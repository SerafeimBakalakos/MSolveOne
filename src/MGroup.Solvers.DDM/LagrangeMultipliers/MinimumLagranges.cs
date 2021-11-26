using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace MGroup.Solvers.DDM.LagrangeMultipliers
{
	public class MinimumLagranges : ICrossPointStrategy
	{
		public (int subdomainPlus, int subdomainMinus)[] ListSubdomainCombinations(IEnumerable<int> subdomainIDs)
		{
			int[] subdomains = subdomainIDs.OrderBy(s => s).ToArray();
			Debug.Assert(subdomains.Length > 1);
			int numCombos = subdomains.Length - 1;
			var combos = new (int subdomainPlus, int subdomainMinus)[numCombos];
			for (int i = 0; i < numCombos; ++i)
			{
				// Lagrange multiplier between subdomains i, i+1. The one with the min ID will be the positive one.
				if (subdomains[i] < subdomains[i+1])
				{
					combos[i] = (subdomains[i], subdomains[i + 1]);
				}
				else
				{
					combos[i] = (subdomains[i + 1], subdomains[i]);
				}
			}
			return combos;
		}
	}
}
