using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Entities;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Materials.Duplicates;

namespace MGroup.XFEM.IsoXFEM
{
   public class Model
    {
        //                      #2
        //     .________________________________.
        //     |                                |
        //     |                                |
        //     |                                |
        //     |                                |
        //     |                                |
        //  #3 |                                |      #1
        //     |                                |
        //     |                                |
        //     |                                |
        //     .________________________________.
        //                    #0
        public enum ConstrainedSide
        {
            Bottomside,
            Rightside,
            Upperside,
            Leftside,
        }
        private readonly ConstrainedSide constrainedSide;
        //public List<Node> nodes = new List<Node>();
		public List<XNode> nodes = new List<XNode>();
        public List<IsoXfemElement2D> elements = new List<IsoXfemElement2D>();       
        public ElasticMaterial2D material;
        public GeometryProperties geometry;        
        public Dictionary<string, int[]> constraintsOfDofs = new Dictionary<string, int[]>();
        public Model(ElasticMaterial2D material, GeometryProperties geometry, ConstrainedSide constrainedSide=ConstrainedSide.Leftside)
        {            
            this.material = material;
            this.geometry = geometry;
            this.constrainedSide = constrainedSide;
        }
		public void CreateElements()
		{
			#region CreateElementsMatlab
			//for (int i = 1; i <= geometry.numberOfElementsX * geometry.numberOfElementsY; i++)
			//{
			//    decimal val = (i - 1) / geometry.numberOfElementsY;
			//    var cl = Decimal.ToDouble(Math.Round(val, MidpointRounding.ToZero)) + 1;
			//    var node1ID = (int)cl - 2 + i;
			//    var node2ID = node1ID + geometry.numberOfElementsY + 1;
			//    var node3ID = node2ID + 1;
			//    var node4ID = node1ID + 1;
			//    var nodesOfElement = new[]
			//    {
			//        nodes[node1ID],
			//        nodes[node2ID],
			//        nodes[node3ID],
			//        nodes[node4ID]
			//    };
			//    var element = new Element(i-1,material, geometry, nodesOfElement);
			//    int[] dofs = new int[8];
			//    for (int j = 0; j < nodesOfElement.Length; j++)
			//    {
			//        dofs[2 * j] = 2 * element.nodesOfElement[j].ID;
			//        dofs[2 * j + 1] = 2 * element.nodesOfElement[j].ID + 1;
			//    }
			//    element.dofsOfElement = dofs;
			//    elements.Add(element);               
			//}
			#endregion
			int el = 0;
			for (int i = 0; i < geometry.numberOfElementsX; i++)
			{
				for (int j = 0; j < geometry.numberOfElementsY; j++)
				{
					var node1ID = el + i;
					var node2ID = node1ID + geometry.numberOfElementsY + 1;
					var node3ID = node2ID + 1;
					var node4ID = node1ID + 1;
					var nodesOfElement = new[]
					{
							nodes[node1ID],
							nodes[node2ID],
							nodes[node3ID],
							nodes[node4ID]
					};
					var element = new IsoXfemElement2D(el, material, geometry, nodesOfElement);
					int[] dofs = new int[8];
					for (int k = 0; k < nodesOfElement.Length; k++)
					{
						dofs[2 * k] = 2 * element.nodesOfElement[k].ID;
						dofs[2 * k + 1] = 2 * element.nodesOfElement[k].ID + 1;
					}
					element.dofsOfElement = dofs;
					elements.Add(element);
					el = el + 1;
				}
			}
		}

		public void CreateNodes()
		{
			#region MakeCoordinatesMatlab
			//for (int i = 1; i <= (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1); i++)
			//{
			//    decimal val = (i - 1) / (geometry.numberOfElementsY + 1);
			//    var cl = Decimal.ToDouble(Math.Round(val, MidpointRounding.ToZero));
			//    var nodeX= cl * geometry.length / geometry.numberOfElementsX;
			//    var md = (i - 1) % (geometry.numberOfElementsY + 1);
			//    var nodeY= md * geometry.height / geometry.numberOfElementsY;
			//    //Add Constrains
			//    var constrainX = false;
			//    var constrainY = false;
			//    switch (constrainedSide)
			//    {
			//        case ConstrainedSide.Bottomside:
			//            if (nodeY==0)
			//            {
			//                constrainX = true;
			//                constrainY = true;
			//            }
			//            break;
			//        case ConstrainedSide.Rightside:
			//            if(nodeX==geometry.length)
			//            {
			//                constrainX = true;
			//                constrainY = true;
			//            }
			//            break;
			//        case ConstrainedSide.Upperside:
			//            if (nodeY==geometry.height)
			//            {
			//                constrainX = true;
			//                constrainY = true;
			//            }
			//            break;
			//        case ConstrainedSide.Leftside:
			//            if (nodeX == 0)
			//            {
			//                constrainX = true;
			//                constrainY = true;
			//            }
			//            break;
			//        default:
			//            break;
			//    }                
			//    var node = new Node(i - 1, nodeX, nodeY,constrainX,constrainY);
			//    nodes.Add(node);
			//}
			#endregion
			var coordX = 0.0;
			var coordY = 0.0;
			int id = 0;
			for (int i = 0; i < (geometry.numberOfElementsX + 1); i++)
			{
				for (int j = 0; j < (geometry.numberOfElementsY + 1); j++)
				{
					var nodeX = coordX;
					var nodeY = coordY;
					//var constrainX = false;
					//var constrainY = false;
					double[] coords = { nodeX, nodeY };
					var node = new XNode(id, coords);
					switch (constrainedSide)
					{
						case ConstrainedSide.Bottomside:
							if (nodeY == 0)
							{
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							}
							break;
						case ConstrainedSide.Rightside:
							if (nodeX == geometry.length)
							{
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							}
							break;
						case ConstrainedSide.Upperside:
							if (nodeY == geometry.height)
							{
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							}
							break;
						case ConstrainedSide.Leftside:
							if (nodeX == 0)
							{
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
								node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							}
							break;
						default:
							break;
					}
					//var node = new Node(id, nodeX, nodeY, constrainX, constrainY);
					nodes.Add(node);
					coordY = coordY + geometry.height / geometry.numberOfElementsY;
					id = id + 1;
				}
				coordY = 0;
				coordX = coordX + geometry.length / geometry.numberOfElementsX;
			}
		}

		public void FindElementsOnNodes()
        {
			foreach (IsoXfemElement2D element in elements)
			{
				foreach (XNode node in element.Nodes) node.ElementsDictionary[element.ID] = element;
			}
			//for (int k = 0; k < nodes.Count; k++)
   //         {
   //             for (int i = 0; i <elements.Count ; i++)
   //             {
   //                 var element = elements[i];
   //                 for (int j = 0; j <element.nodesOfElement.Count ; j++)
   //                 {
   //                     if (element.nodesOfElement[j].ID==k)
   //                     {
   //                         nodes[k].elementsOnNode.Add(elements[i]);                            
   //                     }
   //                 }
   //             }
   //         }
        }

        public void MakeMesh()
        {
            CreateNodes();
            CreateElements();           
            FindElementsOnNodes();
        }

        public void EnumerateDegreesOfFreedom()
        {               
               var fixedDofs = new int[2 * (geometry.numberOfElementsY + 1)];
               var allDofs = new int[2 * nodes.Count];
                for (int i = 0; i < 2 * (geometry.numberOfElementsY + 1); i++)
                {
                    fixedDofs[i] = i; 
                }
                for (int i = 0; i < 2 * nodes.Count; i++)
                {
                    allDofs[i] = i;  
                }
               var freeDofs = ArraysMethods.SetDiff(fixedDofs, allDofs);
            constraintsOfDofs.Add("FreeDofs", freeDofs);
            constraintsOfDofs.Add("FixedDofs", fixedDofs);
            constraintsOfDofs.Add("AllDofs", allDofs);
         }
   }
}
