using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;

namespace MGroup.XFEM.IsoXFEM.Tests
{
   public class Example1
    {
       public static void RunExample1()
        {
            var geometry = new GeometryProperties(40, 20, 1, 40, 20);
            var material = new MaterialProperties(1, 0.3);
            var model = new Model(material, geometry);
            model.MakeMesh();
            model.EnumerateDegreesOfFreedom();
            var nodalLoad = new NodalLoad(geometry, EndLoad.BottomEnd);
            Vector rhs = nodalLoad.CalcRHS();
            ISolver solver = new SkylineLdlSolver();
            var femAnalysis = new FEMAnalysis(model, solver, rhs);
            TopologyOptimization.IsoXfem(model, femAnalysis);
        }
    }
}
