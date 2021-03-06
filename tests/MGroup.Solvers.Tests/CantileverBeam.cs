using System;
using System.Collections.Generic;
using System.Linq;

using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.PlanarElements;
using MGroup.FEM.Entities;
using MGroup.FEM.Structural.Elements;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Discretization.Mesh.Generation;
using MGroup.MSolve.Discretization.Mesh.Generation.Custom;

using MGroup.Solvers.Direct;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Dofs;

// Geometry:
//             | 
//             |
//            \|/ load
// |           V 
// |------------
// |    length

// Cross section:
//     |
//     | 
//    \|/ load
//     .
//   -----
//  |     |
//  |     |
//  |     | <--- height
//  |     |
//  |     |
//   -----
//     ^
//     |     
//   width

namespace MGroup.Solvers.Tests
{
	public class CantileverBeam
	{
		private const double eulerBeamDimensionsRatio = 10.0;
		private const int subdomainID = 0;

		private readonly double length;
		private readonly double height;
		private readonly double width;
		private readonly double endPointLoad;
		private readonly double youngModulus;
		private readonly Node[] endNodes;

		private CantileverBeam(double length, double height, double width, double endPointLoad,
			double youngModulus, Model model, Node[] endNodes)
		{
			this.length = length;
			this.height = height;
			this.width = width;
			this.endPointLoad = endPointLoad;
			this.youngModulus = youngModulus;
			this.Model = model;
			this.endNodes = endNodes;
		}

		public Model Model { get; }

		public double CalculateEndDeflectionWithEulerBeamTheory()
		{
			if ((length < eulerBeamDimensionsRatio * height) || (length < eulerBeamDimensionsRatio * width))
			{
				throw new ArgumentException("In order for Euler beam theory to hold, the beam's length must be at least"
					+ $" {eulerBeamDimensionsRatio} times greater than each of the other two dimensions.");
			}
			double momentOfInertia = 1.0 / 12.0 * width * Math.Pow(height, 3.0);
			return endPointLoad * Math.Pow(length, 3.0) / (3.0 * youngModulus * momentOfInertia);
		}

		public double CalculateAverageEndDeflectionFromSolution(IGlobalVector solution, IVectorValueExtractor resultsExtractor)
		{
			//DofTable subdomainDofs = solver.LinearSystem.DofOrdering.SubdomainDofOrderings[subdomainID].FreeDofs;
			double endDeflectionSum = 0.0;
			int dofsCount = 0;
			foreach (var node in endNodes)
			{
				try
				{
					double displacement = resultsExtractor.ExtractSingleValue(solution, node, StructuralDof.TranslationY);
					++dofsCount;
					endDeflectionSum += displacement;
				}
				catch (KeyNotFoundException)
				{
					// Ignore constrained dofs
				}
			}
			return endDeflectionSum / dofsCount;
		}


		public class Builder
		{
			public double Length { get; set; } = 10.00;
			public double Height { get; set; } = 0.50;
			public double Width { get; set; } = 0.25;
			public double EndPointLoad { get; set; } = 20.0E3;
			public double YoungModulus { get; set; } = 2.1E7;
			public double PoissonRatio { get; set; } = 0.3;

			public CantileverBeam BuildWithBeamElements(int numElements)
			{
				throw new NotImplementedException();
			}

			public CantileverBeam BuildWithQuad4Elements(int numElementsAlongLength, int numElementsAlongHeight)
			{
				// Material and section properties
				double thickness = Width;
				var material = new ElasticMaterial2D(StressState2D.PlaneStress)
				{
					YoungModulus = this.YoungModulus,
					PoissonRatio = this.PoissonRatio
				};
				var dynamicProperties = new DynamicMaterial(1.0, 0.0, 0.0);

				// Model with 1 subdomain
				var model = new Model();
				model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

				// Generate mesh
				var meshGenerator = new UniformMeshGenerator2D<Node>(0.0, 0.0, Length, Height,
					numElementsAlongLength, numElementsAlongHeight);
				(IReadOnlyList<Node> vertices, IReadOnlyList<CellConnectivity<Node>> cells) =
					meshGenerator.CreateMesh((id, x, y, z) => new Node(id: id, x: x, y: y, z: z));

				// Add nodes to the model
				for (int n = 0; n < vertices.Count; ++n) model.NodesDictionary.Add(n, vertices[n]);

				// Add Quad4 elements to the model
				var factory = new ContinuumElement2DFactory(thickness, material, dynamicProperties);
				for (int e = 0; e < cells.Count; ++e)
				{
					ContinuumElement2D element = factory.CreateElement(cells[e].CellType, cells[e].Vertices);
					var elementWrapper = new Element() { ID = e, ElementType = element };
					foreach (Node node in element.Nodes) elementWrapper.AddNode(node);
					model.ElementsDictionary.Add(e, elementWrapper);
					model.SubdomainsDictionary[subdomainID].Elements.Add(elementWrapper);
				}

				// Clamp boundary condition at one end
				double tol = 1E-10; //TODO: this should be chosen w.r.t. the element size along X
				foreach (var node in model.NodesDictionary.Values.Where(node => Math.Abs(node.X) <= tol))
				{
					node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
					node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
				}

				// Apply concentrated load at the other end
				Node[] loadedNodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.X - Length) <= tol).ToArray();
				foreach (var node in loadedNodes)
				{
					model.Loads.Add(new Load() { Amount = EndPointLoad / loadedNodes.Length, Node = node, DOF = StructuralDof.TranslationY });
				}

				return new CantileverBeam(Length, Height, Width, EndPointLoad, YoungModulus, model, loadedNodes);
			}

			public CantileverBeam BuildWithHexa8Elements(int numElementsAlongLength, int numElementsAlongHeight,
				int numElementsAlongWidth)
			{
				throw new NotImplementedException();
			}
		}
	}
}
