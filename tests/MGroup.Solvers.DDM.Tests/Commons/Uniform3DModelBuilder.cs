//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Linq;
//using MGroup.MSolve.Discretization;
//using MGroup.MSolve.Discretization.Loads;
//using MGroup.MSolve.Discretization.Mesh.Generation;
//using MGroup.MSolve.Geometry.Coordinates;
//using MTwin.Engine.Constitutive.Structural;
//using MTwin.Engine.Constitutive.Structural.ContinuumElements;
//using MTwin.Engine.FEM.Entities;
//using MTwin.Engine.FEM.Loading;
//using MTwin.Engine.FEM.Structural.Elements.Continuum;

//namespace MGroup.Solvers.DDM.Tests.Commons
//{
//    public class Uniform3DModelBuilder
//    {
//        public enum BoundaryRegion
//        {
//            MinX, MinY, MinZ, MaxX, MaxY, MaxZ, 
//            MinXMinYMinZ, MinXMinYMaxZ, MinXMaxYMinZ, MinXMaxYMaxZ, MaxXMinYMinZ, MaxXMinYMaxZ, MaxXMaxYMinZ, MaxXMaxYMaxZ,
//            Centroid
//            //TODO: also the lines MinXMinY, MaxXMinZ, etc
//        }

//        private List<(BoundaryRegion region, IDofType dof, double displacement)> prescribedDisplacements;
//        private List<(BoundaryRegion region, IDofType dof, double load)> prescribedLoads;

//        public Uniform3DModelBuilder()
//        {
//            prescribedDisplacements = new List<(BoundaryRegion region, IDofType dof, double displacement)>();
//            prescribedLoads = new List<(BoundaryRegion region, IDofType dof, double load)>();
//        }

//        public double MaxX { get; set; } = 1.0;
//        public double MaxY { get; set; } = 1.0;
//        public double MaxZ { get; set; } = 1.0;
//        public double MinX { get; set; } = 0.0;
//        public double MinY { get; set; } = 0.0;
//        public double MinZ { get; set; } = 0.0;

//        public int NumSubdomainsX { get; set; } = 1;
//        public int NumSubdomainsY { get; set; } = 1;
//        public int NumSubdomainsZ { get; set; } = 1;
//        public int NumTotalElementsX { get; set; } = 1;
//        public int NumTotalElementsY { get; set; } = 1;
//        public int NumTotalElementsZ { get; set; } = 1;

//        public double PoissonRatio { get; set; } = 0.3;
//        public double YoungModulus { get; set; } = 1.0;

//        /// <summary>
//        /// Layout: left to right, then bottom to top. Example for 3x2 subdomains:
//        /// ----------------  
//        /// | E3 | E4 | E5 |
//        /// ----------------
//        /// | E0 | E1 | E2 |
//        /// ----------------
//        /// <see cref="YoungModuliOfSubdomains"/> = {{ E0, E1, E2 },{ E3, E4, E5 }}
//        /// </summary>
//        public double[,,] YoungModuliOfSubdomains { get; set; } = null;

//        public Model BuildModel()
//        {
//            // Generate global mesh
//            double dx = (MaxX - MinX) / NumTotalElementsX;
//            double dy = (MaxY - MinY) / NumTotalElementsY;
//            double dz = (MaxZ - MinZ) / NumTotalElementsZ;
//            double meshTolerance = 1E-10 * Math.Min(dx, dy);
//            var meshGenerator = new UniformMeshGenerator3D<Node>(MinX, MinY, MinZ, MaxX, MaxY, MaxZ,
//                NumTotalElementsX, NumTotalElementsY, NumTotalElementsZ);
//            (IReadOnlyList<Node> vertices, IReadOnlyList<CellConnectivity<Node>> cells) = 
//                meshGenerator.CreateMesh((id, x, y, z) => new Node(id: id, x: x, y:  y, z: z ));

//            // Define subdomain boundaries
//            int numTotalSubdomains = NumSubdomainsX * NumSubdomainsY * NumSubdomainsZ;
//            var boundaries = new Brick[numTotalSubdomains];
//            double subdomainLengthX = (MaxX - MinX) / NumSubdomainsX;
//            double subdomainLengthY = (MaxY - MinY) / NumSubdomainsY;
//            double subdomainLengthZ = (MaxZ - MinZ) / NumSubdomainsZ;
//            for (int k = 0; k < NumSubdomainsZ; ++k)
//            {
//                double minZ = MinZ + k * subdomainLengthZ;
//                double maxZ = MinZ + (k + 1) * subdomainLengthZ;
//                for (int j = 0; j < NumSubdomainsY; ++j)
//                {
//                    double minY = MinY + j * subdomainLengthY;
//                    double maxY = MinY + (j + 1) * subdomainLengthY;
//                    for (int i = 0; i < NumSubdomainsX; ++i)
//                    {
//                        double minX = MinX + i * subdomainLengthX;
//                        double maxX = MinX + (i + 1) * subdomainLengthX;
//                        boundaries[k * NumSubdomainsX * NumSubdomainsY + j * NumSubdomainsX + i] = 
//                            new Brick(minX, minY, minZ, maxX, maxY, maxZ);
//                    }
//                }
//            }

//            // Materials
//            var youngModuli = new double[numTotalSubdomains];
//            if (YoungModuliOfSubdomains == null)
//            {
//                for (int s = 0; s < numTotalSubdomains; ++s) youngModuli[s] = YoungModulus;
//            }
//            else
//            {
//                throw new NotImplementedException();
//                //Debug.Assert(YoungModuliOfSubdomains.GetLength(0) == NumSubdomainsY
//                //    && YoungModuliOfSubdomains.GetLength(1) == NumSubdomainsX, "Materials do not match the subdomain layout");
//                //for (int j = 0; j < NumSubdomainsY; ++j)
//                //{
//                //    for (int i = 0; i < NumSubdomainsX; ++i)
//                //    {
//                //        youngModuli[j * NumSubdomainsX + i] = YoungModuliOfSubdomains[j, i];
//                //    }
//                //}
//            }
//            var dynamicProperties = new DynamicMaterial(1.0, 0.0, 0.0);
//            ElasticMaterial3D[] materials = youngModuli.Select(
//                E => new ElasticMaterial3D() { YoungModulus = E, PoissonRatio = this.PoissonRatio }).ToArray();

//            // Define model, subdomains, nodes
//            var model = new Model();
//            for (int s = 0; s < numTotalSubdomains; ++s) model.SubdomainsDictionary.Add(s, new Subdomain(s));
//            for (int n = 0; n < vertices.Count; ++n) model.NodesDictionary.Add(n, vertices[n]);

//            // Elements
//            ContinuumElement3DFactory[] elementFactories = materials.Select(
//                material => new ContinuumElement3DFactory(material, dynamicProperties)).ToArray();
//            for (int e = 0; e < cells.Count; ++e)
//            {
//                int NumSubdomainsContainingThis = 0;
//                for (int s = 0; s < numTotalSubdomains; ++s)
//                {
//                    if (boundaries[s].Contains(cells[e], meshTolerance))
//                    {
//                        ++NumSubdomainsContainingThis;

//                        // Create the element
//                        ContinuumElement3D element = elementFactories[s].CreateElement(cells[e].CellType, cells[e].Vertices);
//                        var elementWrapper = new Element() { ID = e, ElementType = element };
//                        foreach (Node node in element.Nodes) elementWrapper.AddNode(node);
//                        if (model.ElementsDictionary.ContainsKey(e))
//                        {
//                            throw new Exception($"Element {e} belongs to more than one subdomains.");
//                        }
//                        model.ElementsDictionary.Add(e, elementWrapper);
//                        model.SubdomainsDictionary[s].Elements.Add(elementWrapper);
//                    }
//                }
//                Debug.Assert(NumSubdomainsContainingThis == 1);
//            }

//            // Apply prescribed displacements
//            foreach ((BoundaryRegion region, IDofType dof, double displacement) in prescribedDisplacements)
//            {
//                Node[] nodes = FindBoundaryNodes(region, model, meshTolerance);
//                foreach (Node node in nodes) node.Constraints.Add(new Constraint() { DOF = dof, Amount = displacement });
//            }

//            // Apply prescribed loads
//            foreach ((BoundaryRegion region, IDofType dof, double totalLoad) in prescribedLoads)
//            {
//                Node[] nodes = FindBoundaryNodes(region, model, meshTolerance);
//                double load = totalLoad / nodes.Length;
//                foreach (Node node in nodes) model.Loads.Add(new NodalLoad(node, dof, load));
//            }

//            return model;
//        }

//        /// <summary>
//        /// </summary>
//        /// <param name="load">Will be distributed evenly.</param>
//        public void DistributeLoadAtNodes(BoundaryRegion region, IDofType dof, double load)
//            => prescribedLoads.Add((region, dof, load));

//        public void PrescribeDisplacement(BoundaryRegion region, IDofType dof, double displacement)
//            => prescribedDisplacements.Add((region, dof, displacement));

//        private Node[] FindBoundaryNodes(BoundaryRegion region, Model model, double tol)
//        {
//            IEnumerable<Node> nodes;
//            if (region == BoundaryRegion.MinX) nodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.X - MinX) <= tol);
//            else if (region == BoundaryRegion.MinY) nodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.Y - MinY) <= tol);
//            else if (region == BoundaryRegion.MinZ) nodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.Z - MinZ) <= tol);
//            else if (region == BoundaryRegion.MaxX) nodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.X - MaxX) <= tol);
//            else if (region == BoundaryRegion.MaxY) nodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.Y - MaxY) <= tol);
//            else if (region == BoundaryRegion.MaxZ) nodes = model.NodesDictionary.Values.Where(node => Math.Abs(node.Z - MaxZ) <= tol);
//            else if (region == BoundaryRegion.MinXMinYMinZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node => 
//                    (Math.Abs(node.X - MinX) <= tol) && (Math.Abs(node.Y - MinY) <= tol) && (Math.Abs(node.Z - MinZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MinXMinYMaxZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MinX) <= tol) && (Math.Abs(node.Y - MinY) <= tol) && (Math.Abs(node.Z - MaxZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MinXMaxYMinZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MinX) <= tol) && (Math.Abs(node.Y - MaxY) <= tol) && (Math.Abs(node.Z - MinZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MinXMaxYMaxZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MinX) <= tol) && (Math.Abs(node.Y - MaxY) <= tol) && (Math.Abs(node.Z - MaxZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MaxXMinYMinZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MaxX) <= tol) && (Math.Abs(node.Y - MinY) <= tol) && (Math.Abs(node.Z - MinZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MaxXMinYMaxZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MaxX) <= tol) && (Math.Abs(node.Y - MinY) <= tol) && (Math.Abs(node.Z - MaxZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MaxXMaxYMinZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MaxX) <= tol) && (Math.Abs(node.Y - MaxY) <= tol) && (Math.Abs(node.Z - MinZ) <= tol));
//            }
//            else if (region == BoundaryRegion.MaxXMaxYMaxZ)
//            {
//                nodes = model.NodesDictionary.Values.Where(node =>
//                    (Math.Abs(node.X - MaxX) <= tol) && (Math.Abs(node.Y - MaxY) <= tol) && (Math.Abs(node.Z - MaxZ) <= tol));
//            }
//            else if (region == BoundaryRegion.Centroid)
//            {
//				if ((NumTotalElementsX % 2 != 0) || (NumTotalElementsX % 2 != 0) || (NumTotalElementsZ % 2 != 0))
//				{
//					throw new ArgumentException(
//						"To manipulate the node at the centre, the number of elements in each axis must be even");
//				}

//				double centerX = 0.5 * (MinX + MaxX);
//                double centerY = 0.5 * (MinY + MaxY);
//                double centerZ = 0.5 * (MinZ + MaxZ);
//                var centroid = new CartesianPoint(centerX, centerY, centerZ);

//                // LINQ note: if you call Min() on a sequence of tuples, then the tuple that has minimum Item1 will be returned
//                Node centroidNode = model.NodesDictionary.Values.Select(n => (n.CalculateDistanceFrom(centroid), n)).Min().Item2;
//                nodes = new Node[] { centroidNode };
//            }
//            else throw new Exception("Should not have reached this code");

//            return nodes.ToArray();
//        }

//        private class Brick
//        {
//            private readonly double minX, minY, minZ, maxX, maxY, maxZ;

//            internal Brick(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
//            {
//                this.minX = minX;
//                this.minY = minY;
//                this.minZ = minZ;
//                this.maxX = maxX;
//                this.maxY = maxY;
//                this.maxZ = maxZ;
//            }

//            public bool Contains(CellConnectivity<Node> cell, double tol)
//            {
//                return cell.Vertices.All(node =>
//                     (node.X >= minX - tol) && (node.X <= maxX + tol) 
//                     && (node.Y >= minY - tol) && (node.Y <= maxY + tol)
//                     && (node.Z >= minZ - tol) && (node.Z <= maxZ + tol));
//            }
//        }
//    }
//}
