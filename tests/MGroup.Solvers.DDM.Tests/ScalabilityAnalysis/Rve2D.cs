using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.PlanarElements;
using MGroup.Environments;
using MGroup.FEM.Entities;
using MGroup.FEM.Structural.Elements;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Meshes.Structured;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.Partitioning;
using MGroup.Solvers.DDM.Tests.ExampleModels;

namespace MGroup.Solvers.DDM.Tests.ScalabilityAnalysis
{
	public class Rve2D : IModelBuilder
	{
		public double[] DomainLengthPerAxis { get; set; } = { 2, 2 };

		public int[] NumElementsPerAxis { get; set; }

		public int[] NumSubdomainsPerAxis { get; set; }

		public int[] NumClustersPerAxis { get; set; } = { 1, 1 };

		public int[] SubdomainSizePerElementSize 
		{ 
			get
			{
				var result = new int[2];
				for (int i = 0; i < result.Length; i++)
				{
					if (NumElementsPerAxis[i] % NumSubdomainsPerAxis[i] != 0)
					{
						throw new ArgumentException("Elements per axis must be a multple of subdomains per axis");
					}
					result[i] = NumElementsPerAxis[i] / NumSubdomainsPerAxis[i];
				}
				return result;
			}
		}


		public double YoungModulus { get; set; } = 2.1E7;

		public double[] CenterDisplacement { get; set; } = { 0.01, -0.02 };

		public (IModel model, ICornerDofSelection cornerDofs, ComputeNodeTopology nodeTopology) CreateMultiSubdomainModel()
		{
			var model = CreateSingleSubdomainModel();
			var mesh = new UniformCartesianMesh2D.Builder(new double[2], DomainLengthPerAxis, NumElementsPerAxis)
				.SetMajorAxis(0).BuildMesh();
			var partitioner = new UniformMeshPartitioner2D(mesh, NumSubdomainsPerAxis, NumClustersPerAxis);
			partitioner.Partition(model);
			ModelUtilities.DecomposeIntoSubdomains(model, partitioner.NumSubdomainsTotal, partitioner.GetSubdomainOfElement);

			var topology = new ComputeNodeTopology();
			for (int s = 0; s < partitioner.NumSubdomainsTotal; ++s)
			{
				topology.AddNode(s, partitioner.GetNeighboringSubdomains(s), partitioner.GetClusterOfSubdomain(s));
			}

			return (model, GetCornerDofs(model), topology);
		}

		IModel IModelBuilder.CreateSingleSubdomainModel() => CreateSingleSubdomainModel();


		public (List<int[]> numElements, int[] numSubdomains) GetParametricConfigConstNumSubdomains()
		{
			int[] numSubdomains = { 8, 8 };
			var numElements = new List<int[]>();

			numElements.Add(new int[] { 16, 16 });
			numElements.Add(new int[] { 32, 32 });
			numElements.Add(new int[] { 64, 64 });
			//numElements.Add(new int[] { 128, 128 });
			//numElements.Add(new int[] { 256, 256 });
			//numElements.Add(new int[] { 512, 512 });
			//numElements.Add(new int[] { 768, 768 });

			return (numElements, numSubdomains);
		}

		public (int[] numElements, List<int[]> numSubdomains) GetParametricConfigConstNumElements()
		{
			int[] numElements = { 256, 256 };
			var numSubdomains = new List<int[]>();

			numSubdomains.Add(new int[] { 4, 4 });
			numSubdomains.Add(new int[] { 8, 8 });
			numSubdomains.Add(new int[] { 16, 16 });
			numSubdomains.Add(new int[] { 32, 32 });
			numSubdomains.Add(new int[] { 64, 64 });
			numSubdomains.Add(new int[] { 128, 128 });
			//numSubdomains.Add(new int[] { 256, 256 });

			return (numElements, numSubdomains);
		}

		public (List<int[]> numElements, List<int[]> numSubdomains) GetParametricConfigConstSubdomainPerElementSize()
		{
			var numElements = new List<int[]>();
			var numSubdomains = new List<int[]>();

			//numSubdomains.Add(new int[] { 4, 4 });
			//numSubdomains.Add(new int[] { 8, 8 });
			//numSubdomains.Add(new int[] { 16, 16 });
			//numSubdomains.Add(new int[] { 32, 32 });
			//numSubdomains.Add(new int[] { 64, 64 });
			numSubdomains.Add(new int[] { 96, 96 });
			//numSubdomains.Add(new int[] { 128, 128 });
			//numSubdomains.Add(new int[] { 256, 256 });

			//numElements.Add(new int[] { 32, 32 });
			//numElements.Add(new int[] { 64, 64 });
			//numElements.Add(new int[] { 128, 128 });
			//numElements.Add(new int[] { 256, 256 });
			//numElements.Add(new int[] { 512, 512 });
			numElements.Add(new int[] { 768, 768 });
			//numElements.Add(new int[] { 1024, 1024 });
			//numElements.Add(new int[] { 2048, 2048 });

			return (numElements, numSubdomains);
		}

		private Model CreateSingleSubdomainModel()
		{
			var mesh = new UniformCartesianMesh2D.Builder(new double[2], DomainLengthPerAxis, NumElementsPerAxis)
				.SetMajorAxis(0).BuildMesh();

			var model = new Model();
			model.AllDofs.AddDof(StructuralDof.TranslationX);
			model.AllDofs.AddDof(StructuralDof.TranslationY);
			model.SubdomainsDictionary[0] = new Subdomain(0);

			// Nodes
			foreach ((int id, double[] coords) in mesh.EnumerateNodes())
			{
				model.NodesDictionary[id] = new Node(id, coords[0], coords[1]);
			}

			// Materials
			var material = new ElasticMaterial2D(StressState2D.PlaneStress)
			{
				YoungModulus = this.YoungModulus,
				PoissonRatio = 0.3
			};
			var dynamicProperties = new DynamicMaterial(1.0, 1.0, 1.0);

			// Elements
			double thickness = 1.0;
			var elemFactory = new ContinuumElement2DFactory(thickness, material, dynamicProperties);
			foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
			{
				Node[] nodes = nodeIDs.Select(n => model.NodesDictionary[n]).ToArray();
				var elementType = elemFactory.CreateElement(mesh.CellType, nodes);
				var element = new Element() { ID = elementID, ElementType = elementType };
				foreach (var node in nodes) element.AddNode(node);
				model.ElementsDictionary[element.ID] = element;
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			// Boundary conditions
			double dx = DomainLengthPerAxis[0] / NumElementsPerAxis[0];
			double meshTol = 1E-6 * dx;
			Func<Node, bool> isBoundaryNode = node =>
			{
				double[] x = node.Coordinates;
				for (int d = 0; d < 2; ++d)
				{
					if ((x[d] < meshTol) || (x[d] > DomainLengthPerAxis[d] - meshTol))
					{
						return true;
					}
				}
				return false;
			};
			IEnumerable<Node> boundaryNodes = model.NodesDictionary.Values.Where(isBoundaryNode);
			foreach (Node node in boundaryNodes)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
			}

			double[] center = { 0.5 * DomainLengthPerAxis[0], 0.5 * DomainLengthPerAxis[1] };
			Node centerNode = model.NodesDictionary.Values.First(
				n => (Math.Abs(n.X - center[0]) < meshTol) && (Math.Abs(n.Y - center[1]) < meshTol));
			centerNode.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = CenterDisplacement[0] });
			centerNode.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = CenterDisplacement[1] });
			

			return model;
		}

		private ICornerDofSelection GetCornerDofs(Model model)
		{
			var cornerNodes = new HashSet<int>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				INode[] subdomainCorners = CornerNodeUtilities.FindCornersOfRectangle2D(subdomain);
				foreach (INode node in subdomainCorners)
				{
					if (node.Constraints.Count > 0)
					{
						continue;
					}

					if (node.Subdomains.Count > 1) //TODO for some reason this does not work if > 2. One Krr is singular.
					{
						cornerNodes.Add(node.ID);
					}
				}
			}

			var cornerDofs = new UserDefinedCornerDofSelection();
			foreach (int node in cornerNodes)
			{
				cornerDofs.AddCornerNode(node);
			}
			return cornerDofs;
		}
	}
}
