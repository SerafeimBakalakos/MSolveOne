using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.Dofs;

namespace MGroup.Solvers.DDM.Tests.ScalabilityAnalysis
{
	public interface IModelBuilder
	{
		double[] DomainLengthPerAxis { get; set; }

		int[] NumElementsPerAxis { get; set; }

		int[] NumSubdomainsPerAxis { get; set; }

		int[] SubdomainSizePerElementSize { get; }

		(IModel model, ICornerDofSelection cornerDofs, ComputeNodeTopology nodeTopology) CreateMultiSubdomainModel();

		IModel CreateSingleSubdomainModel();

		(List<int[]> numElements, int[] numSubdomains) GetParametricConfigConstNumSubdomains();

		(int[] numElements, List<int[]> numSubdomains) GetParametricConfigConstNumElements();

		(List<int[]> numElements, List<int[]> numSubdomains) GetParametricConfigConstSubdomainPerElementSize();
	}
}
