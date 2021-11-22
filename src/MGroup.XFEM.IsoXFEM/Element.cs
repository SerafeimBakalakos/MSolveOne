using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.IsoXFEM.ElementStructuralStiffnessComputations;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.IsoXFEM
{

   public class Element
    {
        public enum PhaseElement
        {
            solidElement,
            voidElement,
            boundaryElement
        }
        public int ID { get; }
        public MaterialProperties material;
        public GeometryProperties geometry;
        public List<Node> nodesOfElement = new List<Node>();
        public IEnumerable<Node> NodesOfElement {get => nodesOfElement;}
        public int[] dofsOfElement;
        public readonly Matrix elasticityMatrix;
        private readonly double lengthOfElement;
        private readonly double heigthOfElement;
        public double areaOfElement;
        public Matrix stiffnessOfElement;
        private static IMatrixView defaultStiffness;
        public PhaseElement phaseElement;
        public  Matrix coordinatesOfElement;
        private static IElementStructuralStiffnessComputation elementStructuralStiffnessComputation;

        public Element(int id, MaterialProperties material, GeometryProperties geometry, IEnumerable<Node> nodesOfElement)
        {
            ID = id >= 0 ? id : -id;
            this.geometry = geometry;
            this.material = material;
            this.nodesOfElement.AddRange(nodesOfElement);
            elasticityMatrix = Matrix.CreateFromArray(new double[,]
                {{ 1* (material.YoungModulus / (1-Math.Pow( material.PoissonRatio, 2))), material.PoissonRatio*(material.YoungModulus /(1-Math.Pow( material.PoissonRatio, 2))), 0},
                { material.PoissonRatio*(material.YoungModulus / (1-Math.Pow( material.PoissonRatio, 2))),1*(material.YoungModulus / (1-Math.Pow(material.PoissonRatio, 2))),0},{ 0,0,((1 - material.PoissonRatio)/2)*(material.YoungModulus /(1-Math.Pow(material.PoissonRatio, 2)))} });
            lengthOfElement = geometry.length / geometry.numberOfElementsX;
            heigthOfElement = geometry.height / geometry.numberOfElementsY;
            areaOfElement = lengthOfElement * heigthOfElement;
            coordinatesOfElement = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { lengthOfElement, 0 }, { lengthOfElement, heigthOfElement }, { 0, heigthOfElement } });
            elementStructuralStiffnessComputation = new ElementStructuralStiffnessFEMComputation();
            if(defaultStiffness==null)
            {
                defaultStiffness = elementStructuralStiffnessComputation.ElementStructuralStiffnessComputation(coordinatesOfElement, elasticityMatrix, geometry.thickness);
            }
            stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
        }
        
        public void CalcStiffnessAndArea(Vector elementLevelSet)
        {
            if (elementLevelSet.Min() >= 0)
            {
                phaseElement = PhaseElement.solidElement;               
                stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
                areaOfElement = lengthOfElement * heigthOfElement;
            }
            else if (elementLevelSet.Max() <= 0)
            {
                phaseElement = PhaseElement.voidElement;               
                stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
                stiffnessOfElement.ScaleIntoThis(0.0001);
                areaOfElement = 0.00;
            }
            else
            {
                phaseElement = PhaseElement.boundaryElement;
                var integration = new XFEMIntegration();
                integration.MeshAndAreaOfSubElement(coordinatesOfElement, elementLevelSet);
                var elementStructuralStiffnessComputation = new ElementStructuralStiffnessXFEMComputation(integration.coordinatesOfBoundaryElement, integration.connectionOfBoundaryElement);
                areaOfElement = integration.areaBoundaryElement;
                var stiffness = elementStructuralStiffnessComputation.ElementStructuralStiffnessComputation(coordinatesOfElement, elasticityMatrix, geometry.thickness);
                stiffnessOfElement = stiffness;
            }
        }
        
    }
}

