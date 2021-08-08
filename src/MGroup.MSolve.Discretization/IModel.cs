using System.Collections.Generic;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;

namespace MGroup.MSolve.Discretization
{
	public interface IModel
	{
		ActiveDofs AllDofs { get; }

		int NumSubdomains { get; }

		void ConnectDataStructures();

		IEnumerable<DirichletElementLoad> EnumerateDirichletBoundaryConditions(int subdomainID);

		//TODO: I could take all elements and filter them with an IPartitioner object. 
		//		Same goes for all entities that are based on INode or IElement. We can have INodalVectorContributor: INodeBasedEntity
		//		and IElementVectorContributor: IElementBasedEntity and always access them inside IEnumerable<>. 
		//		This will completely decouple model from subdomains, at least when all entities live in the same machine.
		IEnumerable<IElement> EnumerateElements(int subdomainID);

		IEnumerable<Load> EnumerateNodalLoads(int subdomainID);

		IEnumerable<INode> EnumerateNodes();

		IEnumerable<ISubdomain> EnumerateSubdomains();

		INode GetNode(int nodeID);

		ISubdomain GetSubdomain(int subdomainID);

		void SaveMaterialState();

		void ScaleConstraints(double scalingFactor);
	}
}
