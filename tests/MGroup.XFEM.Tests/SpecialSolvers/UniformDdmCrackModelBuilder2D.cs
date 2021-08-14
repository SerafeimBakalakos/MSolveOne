using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Structured;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.Partitioning;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;

//TODO: Allow option for prescribed displacement/load at corners or specific node index/ID.
namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public class UniformDdmCrackModelBuilder2D
	{
		public enum BoundaryRegion
		{
			LeftSide, RightSide, UpperSide, LowerSide, UpperLeftCorner, UpperRightCorner, LowerLeftCorner, LowerRightCorner, 
			Center
		}

		private List<(BoundaryRegion region, IDofType dof, double displacement)> prescribedDisplacements;
		private List<(BoundaryRegion region, IDofType dof, double load)> prescribedLoads;

		public UniformDdmCrackModelBuilder2D()
		{
			prescribedDisplacements = new List<(BoundaryRegion region, IDofType dof, double displacement)>();
			prescribedLoads = new List<(BoundaryRegion region, IDofType dof, double load)>();

			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			BulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
		}

		public CrackElementIntegrationStrategy BulkIntegration { get; set; }

		public double[] MinCoords { get; set; } = { -1.0, -1.0 };

		public double[] MaxCoords { get; set; } = { 1.0, 1.0 };

		public int[] NumElementsTotal { get; set; } = { 1, 1 };

		public int[] NumSubdomains { get; set; } = { 1, 1 };

		public int[] NumClusters { get; set; } = { 1, 1 };

		public double Thickness { get; set; } = 1.0;

		public IFractureMaterialField MaterialField { get; set; } = new HomogeneousFractureMaterialField2D(1, 0.3, 1.0, true);

		public bool FindConformingSubcells { get; set; } = true;

		public (XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology) BuildMultiSubdomainModel()
		{
			XModel<IXCrackElement> model = BuildSingleSubdomainModel();
			UniformCartesianMesh2D mesh = BuildMesh();
			var partitioner = new UniformMeshPartitioner2D(mesh, NumSubdomains, NumClusters);
			partitioner.Partition(model);
			ModelUtilities.DecomposeIntoSubdomains(model, partitioner.NumSubdomainsTotal, partitioner.GetSubdomainOfElement);

			var topology = new ComputeNodeTopology();
			for (int s = 0; s < partitioner.NumSubdomainsTotal; ++s)
			{
				topology.AddNode(s, partitioner.GetNeighboringSubdomains(s), partitioner.GetClusterOfSubdomain(s));
			}

			return (model, topology);
		}

		public XModel<IXCrackElement> BuildSingleSubdomainModel()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[0] = new XSubdomain<IXCrackElement>(0);

			UniformCartesianMesh2D mesh = BuildMesh();

			// Nodes
			foreach ((int id, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[id] = new XNode(id, coords);
			}

			// Elements
			var elemFactory = new XCrackElementFactory2D(MaterialField, Thickness, BulkIntegration);
			foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
			{
				XNode[] nodes = nodeIDs.Select(n => model.Nodes[n]).ToArray();
				var element = elemFactory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[element.ID] = element;
				model.Subdomains[0].Elements.Add(element);
			}

			ApplyBoundaryConditions(model);

			model.FindConformingSubcells = this.FindConformingSubcells;
			return model;
		}

		/// <summary>
		/// If there are nodes belonging to <paramref name="region"/> taht are constrained along <paramref name="dof"/>, then 
		/// they will not be loaded, but the total <paramref name="load"/> will be divided by the total count of nodes, even the
		/// ones that will not be loaded.
		/// If there are multiple loads at the same (node, dof) then their sum will be used.
		/// </summary>
		/// <param name="load">Will be distributed evenly.</param>
		public void DistributeLoadAtNodes(BoundaryRegion region, IDofType dof, double load)
			=> prescribedLoads.Add((region, dof, load));

		public void PrescribeDisplacement(BoundaryRegion region, IDofType dof, double displacement)
			=> prescribedDisplacements.Add((region, dof, displacement));

		public static IEnumerable<INode> FindCornerNodes(ISubdomain subdomain, int minCornerNodeMultiplicity = 2)
		{
			var cornerNodes = new List<INode>();
			INode[] subdomainCorners = CornerNodeUtilities.FindCornersOfRectangle2D(subdomain);
			foreach (INode node in subdomainCorners)
			{
				if (node.Constraints.Count > 0) //TODO: allow only some dofs to be constrained
				{
					continue;
				}

				if (node.Subdomains.Count >= minCornerNodeMultiplicity)
				{
					cornerNodes.Add(node);
				}
			}
			return cornerNodes;
		}

		private void ApplyBoundaryConditions(XModel<IXCrackElement> model)
		{
			double dx = (MaxCoords[0] - MinCoords[0]) / NumElementsTotal[0];
			double dy = (MaxCoords[1] - MinCoords[1]) / NumElementsTotal[1];
			double meshTolerance = 1E-10 * Math.Min(dx, dy);

			// Apply prescribed displacements
			foreach ((BoundaryRegion region, IDofType dof, double displacement) in prescribedDisplacements)
			{
				XNode[] nodes = FindBoundaryNodes(region, model, meshTolerance);
				foreach (XNode node in nodes)
				{
					// Search if there is already a prescribed displacement at this dof
					bool isFreeDof = true;
					foreach (Constraint constraint in node.Constraints)
					{
						if (constraint.DOF == dof)
						{
							if (constraint.Amount != displacement)
							{
								throw new Exception($"At node {node.ID}, dof = {dof}, " +
									$"both u = {displacement} and u = {constraint.Amount} have been prescribed.");
							}
							else
							{
								isFreeDof = false;

								// DO NOT break here. Let the loop iterate all constraints, since the expection above may be 
								// thrown for another constraint.
								//break; 
							}
						}
					}

					if (isFreeDof)
					{
						node.Constraints.Add(new Constraint() { DOF = dof, Amount = displacement });
					}
				}
			}

			// Apply prescribed loads
			foreach ((BoundaryRegion region, IDofType dof, double totalLoad) in prescribedLoads)
			{
				XNode[] nodes = FindBoundaryNodes(region, model, meshTolerance);
				double load = totalLoad / nodes.Length;
				foreach (XNode node in nodes)
				{
					// Search if there is already a prescribed displacement at this dof
					bool isFreeDof = true;
					foreach (Constraint constraint in node.Constraints)
					{
						if (constraint.DOF == dof)
						{
							isFreeDof = false;
							break;
						}
					}

					if (isFreeDof)
					{
						model.NodalLoads.Add(new Load { Node = node, DOF = dof, Amount = load });
					}
				}
			}
		}

		private UniformCartesianMesh2D BuildMesh()
		{
			return new UniformCartesianMesh2D.Builder(MinCoords, MaxCoords, NumElementsTotal).SetMajorAxis(0).BuildMesh();
		}

		private XNode[] FindBoundaryNodes(BoundaryRegion region, IXModel model, double tol)
		{
			double minX = MinCoords[0], minY = MinCoords[1], maxX = MaxCoords[0], maxY = MaxCoords[1]; // for brevity

			IEnumerable<XNode> allNodes = model.Nodes.Values;
			IEnumerable<XNode> nodes;
			if (region == BoundaryRegion.LeftSide)
			{
				nodes = allNodes.Where(node => Math.Abs(node.X - minX) <= tol);
			}
			else if (region == BoundaryRegion.RightSide)
			{
				nodes = allNodes.Where(node => Math.Abs(node.X - maxX) <= tol);
			}
			else if (region == BoundaryRegion.LowerSide)
			{
				nodes = allNodes.Where(node => Math.Abs(node.Y - minY) <= tol);
			}
			else if (region == BoundaryRegion.UpperSide)
			{
				nodes = allNodes.Where(node => Math.Abs(node.Y - maxY) <= tol);
			}
			else if (region == BoundaryRegion.LowerLeftCorner)
			{
				nodes = allNodes.Where(node => (Math.Abs(node.X - minX) <= tol) && (Math.Abs(node.Y - minY) <= tol));
			}
			else if (region == BoundaryRegion.LowerRightCorner)
			{
				nodes = allNodes.Where(node => (Math.Abs(node.X - maxX) <= tol) && (Math.Abs(node.Y - minY) <= tol));
			}
			else if (region == BoundaryRegion.UpperLeftCorner)
			{
				nodes = allNodes.Where(node => (Math.Abs(node.X - minX) <= tol) && (Math.Abs(node.Y - maxY) <= tol));
			}
			else if (region == BoundaryRegion.UpperRightCorner)
			{
				nodes = allNodes.Where(node => (Math.Abs(node.X - maxX) <= tol) && (Math.Abs(node.Y - maxY) <= tol));
			}
			else if (region == BoundaryRegion.Center)
			{
				if ((NumElementsTotal[0] % 2 != 0) || (NumElementsTotal[1] % 2 != 0))
				{
					throw new ArgumentException(
						"To manipulate the node at the centre, the number of elements in each axis must be even");
				}

				double centerX = 0.5 * (MinCoords[0] + MaxCoords[0]);
				double centerY = 0.5 * (MinCoords[1] + MaxCoords[1]);
				nodes = allNodes.Where(node => (Math.Abs(node.X - centerX) <= tol) && (Math.Abs(node.Y - centerY) <= tol));
				Debug.Assert(nodes.Count() == 1);
			}
			else throw new Exception("Should not have reached this code");

			return nodes.ToArray();
		}
	}
}
