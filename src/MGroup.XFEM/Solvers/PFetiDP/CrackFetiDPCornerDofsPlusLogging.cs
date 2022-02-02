using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Solvers.PFetiDP
{
	public class CrackFetiDPCornerDofsPlusLogging : CrackFetiDPCornerDofs
	{
		public CrackFetiDPCornerDofsPlusLogging(IComputeEnvironment environment, IXModel model, 
			IEnumerable<IDofType> standardCornerDofs, Func<ISubdomain, IEnumerable<INode>> getStandardCornerNodesOfSubdomain)
			: base(environment, model, standardCornerDofs, getStandardCornerNodesOfSubdomain)
		{
		}


		public bool HasEnrCornerDofs(XNode node)
		{
			return (node.Subdomains.Count > 1) && node.IsEnriched;
		}

		public bool HasStdCornerDofs(INode node)
		{
			int numSubdomainsWithCornerNode = 0;
			foreach (int subdomainID in node.Subdomains)
			{
				if (standardCornerNodes[subdomainID].Contains(node.ID))
				{
					++numSubdomainsWithCornerNode;
				}
			}
			if (numSubdomainsWithCornerNode == node.Subdomains.Count)
			{
				return true;
			}
			else
			{
				Debug.Assert(numSubdomainsWithCornerNode == 0, $"Node {node.ID} is corner only in some of its subdomains");
				return false;
			}
		}
	}
}
