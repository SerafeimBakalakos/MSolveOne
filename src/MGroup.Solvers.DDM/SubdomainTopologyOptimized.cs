using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;

//TODO: Needs a better name
namespace MGroup.Solvers.DDM
{
	/// <remarks>
	/// Similar to <see cref="SubdomainTopologyGeneral"/>, but assumes that all subdomains have the same dofs at their common 
	/// nodes, thus many operations can be done more efficiently and simply. 
	/// </remarks>
	public class SubdomainTopologyOptimized : SubdomainTopologyGeneral
	{
		public override void FindCommonDofsBetweenSubdomains()
		{
			environment.DoPerNode(subdomainID =>
			{
				Dictionary<int, DofSet> commonDofs = FindLocalSubdomainDofsAtCommonNodes(subdomainID);
				commonDofsBetweenSubdomains[subdomainID] = commonDofs;
			});
		}

		public override void RefindCommonDofsBetweenSubdomains(Func<int, bool> isModifiedSubdomain)
		{
			environment.DoPerNode(subdomainID =>
			{
				if (isModifiedSubdomain(subdomainID))
				{
					#region log
					//Debug.WriteLine($"Finding common dofs of subdomain {subdomainID} with its neighbors");
					//Console.WriteLine($"Finding common dofs of subdomain {subdomainID} with its neighbors");
					#endregion

					Dictionary<int, DofSet> commonDofs = FindLocalSubdomainDofsAtCommonNodes(subdomainID);
					commonDofsBetweenSubdomains[subdomainID] = commonDofs;
				}
				else
				{
					Debug.Assert(commonDofsBetweenSubdomains.ContainsKey(subdomainID));
				}
			});
		}
	}
}
