using System;
using System.Collections.Generic;
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
		private readonly string outputDirectory;
		private int iteration = 0;
		
		public CrackFetiDPCornerDofsPlusLogging(IComputeEnvironment environment, IXModel model, 
			IEnumerable<IDofType> standardCornerDofs, Func<ISubdomain, IEnumerable<INode>> getStandardCornerNodesOfSubdomain,
			string outputDirectory)
			: base(environment, model, standardCornerDofs, getStandardCornerNodesOfSubdomain)
		{
			this.outputDirectory = outputDirectory;
		}


		public bool HasEnrCornerDofs(XNode node)
		{
			return (node.Subdomains.Count > 1) && node.IsEnriched;
		}

		public bool HasStdCornerDofs(INode node)
		{
			return standardCornerNodes[node.Subdomains.First()].Contains(node.ID);
		}
	}
}
