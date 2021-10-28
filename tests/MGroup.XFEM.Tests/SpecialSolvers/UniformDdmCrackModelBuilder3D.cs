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
	public class UniformDdmCrackModelBuilder3D
	{
		public enum BoundaryRegion
		{
			MinX, MinY, MinZ, MaxX, MaxY, MaxZ,
			MinXMinYMinZ, MinXMinYMaxZ, MinXMaxYMinZ, MinXMaxYMaxZ, MaxXMinYMinZ, MaxXMinYMaxZ, MaxXMaxYMinZ, MaxXMaxYMaxZ,
			Centroid
			//TODO: also the lines MinXMinY, MaxXMinZ, etc
		}

		private List<(BoundaryRegion region, IDofType dof, double displacement)> prescribedDisplacements;
		private List<(BoundaryRegion region, IDofType dof, double load)> prescribedLoads;

		public UniformDdmCrackModelBuilder3D()
		{
			prescribedDisplacements = new List<(BoundaryRegion region, IDofType dof, double displacement)>();
			prescribedLoads = new List<(BoundaryRegion region, IDofType dof, double load)>();

			var enrichedIntegration = new IntegrationWithNonconformingHexa3D(8, GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2));
			BulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
		}

		public Action<XModel<IXCrackElement>> ApplyBoundaryConditions { get; set; } = null;

		public CrackElementIntegrationStrategy BulkIntegration { get; set; }

		public double[] MinCoords { get; set; } = { -1.0, -1.0, -1.0 };

		public double[] MaxCoords { get; set; } = { 1.0, 1.0, 1.0 };

		public int[] NumElementsTotal { get; set; } = { 1, 1, 1 };

		public int[] NumSubdomains { get; set; } = { 1, 1, 1 };

		public int[] NumClusters { get; set; } = { 1, 1, 1 };

		public IFractureMaterialField MaterialField { get; set; } = new HomogeneousFractureMaterialField3D(1, 0.3);

		public bool FindConformingSubcells { get; set; } = true;

		public (XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology) BuildMultiSubdomainModel()
		{
			XModel<IXCrackElement> model = BuildSingleSubdomainModel();
			UniformCartesianMesh3D mesh = BuildMesh();
			var partitioner = new UniformMeshPartitioner3D(mesh, NumSubdomains, NumClusters);
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
			var model = new XModel<IXCrackElement>(3);
			model.Subdomains[0] = new XSubdomain<IXCrackElement>(0);
			UniformCartesianMesh3D mesh = BuildMesh();

			// Nodes
			foreach ((int id, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[id] = new XNode(id, coords);
			}

			// Elements
			var elemFactory = new XCrackElementFactory3D(MaterialField, BulkIntegration);
			foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
			{
				XNode[] nodes = nodeIDs.Select(n => model.Nodes[n]).ToArray();
				var element = elemFactory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[element.ID] = element;
				model.Subdomains[0].Elements.Add(element);
			}

			ApplyBCs(model);

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

		public static IEnumerable<INode> FindCornerNodes(ISubdomain subdomain, int minCornerNodeMultiplicity = 3)
		{
			var cornerNodes = new List<INode>();
			INode[] subdomainCorners = CornerNodeUtilities.FindCornersOfBrick3D(subdomain);
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

		private void ApplyBCs(XModel<IXCrackElement> model)
		{
			if (ApplyBoundaryConditions != null)
			{
				ApplyBoundaryConditions(model);
				return;
			}

			double dx = (MaxCoords[0] - MinCoords[0]) / NumElementsTotal[0];
			double dy = (MaxCoords[1] - MinCoords[1]) / NumElementsTotal[1];
			double dz = (MaxCoords[2] - MinCoords[2]) / NumElementsTotal[1];
			double meshTolerance = 1E-10 * Math.Min(dx, Math.Min(dy, dz));

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

		private UniformCartesianMesh3D BuildMesh()
		{
			return new UniformCartesianMesh3D.Builder(MinCoords, MaxCoords, NumElementsTotal)
				.SetMajorMinorAxisDefault()
				.BuildMesh();
		}

		private XNode[] FindBoundaryNodes(BoundaryRegion region, XModel<IXCrackElement> model, double tol)
		{
			double minX = MinCoords[0], minY = MinCoords[1], minZ = MinCoords[2];
			double maxX = MaxCoords[0], maxY = MaxCoords[1], maxZ = MaxCoords[2];
			IEnumerable<XNode> nodes;
			if (region == BoundaryRegion.MinX)
			{
				nodes = model.Nodes.Values.Where(node => Math.Abs(node.X - minX) <= tol);
			}
			else if (region == BoundaryRegion.MinY)
			{
				nodes = model.Nodes.Values.Where(node => Math.Abs(node.Y - minY) <= tol);
			}
			else if (region == BoundaryRegion.MinZ)
			{
				nodes = model.Nodes.Values.Where(node => Math.Abs(node.Z - minZ) <= tol);
			}
			else if (region == BoundaryRegion.MaxX)
			{
				nodes = model.Nodes.Values.Where(node => Math.Abs(node.X - maxX) <= tol);
			}
			else if (region == BoundaryRegion.MaxY)
			{
				nodes = model.Nodes.Values.Where(node => Math.Abs(node.Y - maxY) <= tol);
			}
			else if (region == BoundaryRegion.MaxZ)
			{
				nodes = model.Nodes.Values.Where(node => Math.Abs(node.Z - maxZ) <= tol);
			}
			else if (region == BoundaryRegion.MinXMinYMinZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - minX) <= tol) && (Math.Abs(node.Y - minY) <= tol) && (Math.Abs(node.Z - minZ) <= tol));
			}
			else if (region == BoundaryRegion.MinXMinYMaxZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - minX) <= tol) && (Math.Abs(node.Y - minY) <= tol) && (Math.Abs(node.Z - maxZ) <= tol));
			}
			else if (region == BoundaryRegion.MinXMaxYMinZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - minX) <= tol) && (Math.Abs(node.Y - maxY) <= tol) && (Math.Abs(node.Z - minZ) <= tol));
			}
			else if (region == BoundaryRegion.MinXMaxYMaxZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - minX) <= tol) && (Math.Abs(node.Y - maxY) <= tol) && (Math.Abs(node.Z - maxZ) <= tol));
			}
			else if (region == BoundaryRegion.MaxXMinYMinZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - maxX) <= tol) && (Math.Abs(node.Y - minY) <= tol) && (Math.Abs(node.Z - minZ) <= tol));
			}
			else if (region == BoundaryRegion.MaxXMinYMaxZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - maxX) <= tol) && (Math.Abs(node.Y - minY) <= tol) && (Math.Abs(node.Z - maxZ) <= tol));
			}
			else if (region == BoundaryRegion.MaxXMaxYMinZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - maxX) <= tol) && (Math.Abs(node.Y - maxY) <= tol) && (Math.Abs(node.Z - minZ) <= tol));
			}
			else if (region == BoundaryRegion.MaxXMaxYMaxZ)
			{
				nodes = model.Nodes.Values.Where(node =>
					(Math.Abs(node.X - maxX) <= tol) && (Math.Abs(node.Y - maxY) <= tol) && (Math.Abs(node.Z - maxZ) <= tol));
			}
			else if (region == BoundaryRegion.Centroid)
			{
				if ((NumElementsTotal[0] % 2 != 0) || (NumElementsTotal[1] % 2 != 0) || (NumElementsTotal[2] % 2 != 0))
				{
					throw new ArgumentException(
						"To manipulate the node at the centre, the number of elements in each axis must be even");
				}

				double centerX = 0.5 * (minX + maxX);
				double centerY = 0.5 * (minY + maxY);
				double centerZ = 0.5 * (minZ + maxZ);
				var centroid = new double[] { centerX, centerY, centerZ };

				// LINQ note: if you call Min() on a sequence of tuples, then the tuple that has minimum Item1 will be returned
				XNode centroidNode = model.Nodes.Values.Select(n => (n.CalculateDistanceFrom(centroid), n)).Min().Item2;
				nodes = new XNode[] { centroidNode };
			}
			else throw new Exception("Should not have reached this code");

			return nodes.ToArray();
		}
	}
}
