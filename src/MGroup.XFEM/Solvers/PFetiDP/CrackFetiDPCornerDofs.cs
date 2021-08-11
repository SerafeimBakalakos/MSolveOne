using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PFetiDP
{
	public class CrackFetiDPCornerDofs : ICornerDofSelection
	{
		private readonly IComputeEnvironment environment;
		private readonly IXModel model;
		private readonly HashSet<int> standardCornerDofs;
		private readonly Dictionary<int, HashSet<int>> standardCornerNodes;

		public CrackFetiDPCornerDofs(IComputeEnvironment environment, IXModel model, IEnumerable<IDofType> standardCornerDofs, 
			Func<ISubdomain, IEnumerable<INode>> getStandardCornerNodesOfSubdomain)
		{
			this.environment = environment;
			this.model = model;
			this.standardCornerDofs = new HashSet<int>(standardCornerDofs.Select(dof => model.AllDofs.GetIdOfDof(dof)));
			this.standardCornerNodes = environment.CalcNodeData(s =>
			{
				ISubdomain subdomain = model.GetSubdomain(s);
				IEnumerable<INode> cornerNodes = getStandardCornerNodesOfSubdomain(subdomain);
				return new HashSet<int>(cornerNodes.Select(node => node.ID));
			});
		}

		public bool IsCornerDof(int subdomainID, int nodeID, int dofID)
		{
			HashSet<int> subdomainCornerNodes = standardCornerNodes[subdomainID];
			if (subdomainCornerNodes.Contains(nodeID))
			{
				// Regular corner dofs, at the standard dofs of the user-defined corner nodes.
				if (standardCornerDofs.Contains(dofID))
				{
					return true;
				}
			}
			else
			{
				// Extra corner dofs used to avoid mechanisms due to a crack dividing the subdomain into 2 parts, by entering 
				// at one point and exiting at another. The linear dependence is caused when boundary nodes have dofs enriched by 
				// a step enrichment or the 1st of the 4 crack tip enrichments.  
				IDofType dof = model.AllDofs.GetDofWithId(dofID);
				if ((dof is CrackStepEnrichment) || (dof is IsotropicBrittleTipEnrichments2D.Func0))
				{
					XNode node = model.Nodes[nodeID];
					if (node.Subdomains.Count > 1)
					{
						// TODO: Actually I should investigate if the enrichment is defined by a crack that enters and then exits 
						//		the subdomain. Otherwise it would not create mechanisms. If the enrichments identified here
						//		are caused by different cracks that do not individually create mechanisms, then the corresponding 
						//		dofs should not be considered as corner dofs.
						return true;
					}
				}
			}

			return false;
		}
	}
}
