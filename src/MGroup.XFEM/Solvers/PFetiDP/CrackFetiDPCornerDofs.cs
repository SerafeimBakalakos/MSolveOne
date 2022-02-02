using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PFetiDP
{
	public class CrackFetiDPCornerDofs : ICornerDofSelection
	{
		//TODO: Use StrategyPattern instead of this monstrocity
		/// <summary>
		/// Strategy of selecting corner dofs of a boundary-enriched node: 
		/// </summary>
		public enum Strategy
		{
			AllDofs, HeavisideAndAllTipDofs, HeavisideAndFirstTipDofs, StandardDofs
		}

		public static Strategy strategy = Strategy.HeavisideAndAllTipDofs;

		protected readonly IComputeEnvironment environment;
		protected readonly IXModel model;
		protected readonly HashSet<int> standardCornerDofs;
		protected readonly Dictionary<int, HashSet<int>> standardCornerNodes;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="environment"></param>
		/// <param name="model"></param>
		/// <param name="standardCornerDofs"></param>
		/// <param name="getStandardCornerNodesOfSubdomain"></param>
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

		public void AddStdCornerNode(int subdomainID, int nodeID)
		{
			environment.DoPerNode(s =>
			{
				if (s == subdomainID)
				{
					standardCornerNodes[s].Add(nodeID);
				}
			});
		}

		public bool IsCornerDof(int subdomainID, int nodeID, int dofID)
		{
			HashSet<int> subdomainCornerNodes = standardCornerNodes[subdomainID];
			if (subdomainCornerNodes.Contains(nodeID))
			{
				// Regular corner dofs, at the standard dofs of the user-defined corner nodes.
				return true;
				//if (standardCornerDofs.Contains(dofID))
				//{
				//	return true;
				//}
			}

			// Extra corner dofs used to avoid mechanisms due to a crack dividing the subdomain into 2 parts, by entering 
			// at one point and exiting at another. The linear dependence is caused when boundary nodes have dofs enriched by 
			// a step enrichment or the 1st of the 4 crack tip enrichments.  

			if (strategy == Strategy.AllDofs)
			{
				XNode node = model.Nodes[nodeID];
				if ((node.Subdomains.Count > 1) && node.IsEnriched) //TODO: With geometric tip enrichment this will return a huge number of nodes
				{
					return true;
				}
			}
			else if (strategy == Strategy.HeavisideAndAllTipDofs)
			{
				IDofType dof = model.AllDofs.GetDofWithId(dofID);
				if (dof is EnrichedDof enrichedDof)
				{
					IEnrichmentFunction enrichment = enrichedDof.Enrichment;
					if ((enrichment is IStepEnrichment) || (enrichment is ICrackTipEnrichment))
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
			}
			else if (strategy == Strategy.HeavisideAndFirstTipDofs)
			{
				IDofType dof = model.AllDofs.GetDofWithId(dofID);
				if (dof is EnrichedDof enrichedDof)
				{
					IEnrichmentFunction enrichment = enrichedDof.Enrichment;
					if ((enrichment is IStepEnrichment) 
						|| (enrichment is IsotropicBrittleTipEnrichments2D.Func0)
						|| (enrichment is IsotropicBrittleTipEnrichments2DCaching.Func0)
						|| (enrichment is IsotropicBrittleTipEnrichments_v2.Func0))
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
			}
			else if (strategy == Strategy.StandardDofs)
			{
				XNode node = model.Nodes[nodeID];
				if ((node.Subdomains.Count > 1) && node.IsEnriched) //TODO: With geometric tip enrichment this will return a huge number of nodes
				{
					if (standardCornerDofs.Contains(dofID))
					{
						return true;
					}
				}
			}

			return false;
		}
	}
}
