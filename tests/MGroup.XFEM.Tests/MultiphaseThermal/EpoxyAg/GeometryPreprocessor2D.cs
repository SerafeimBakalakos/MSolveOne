using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Tests.MultiphaseThermal.EpoxyAg
{
	public class GeometryPreprocessor2D : GeometryPreprocessorBase
	{
		public GeometryPreprocessor2D(XModel<IXMultiphaseElement> physicalModel) : base(physicalModel)
		{
			MinCoordinates = new double[] { -1.0, -1.0};
			MaxCoordinates = new double[] { +1.0, +1.0 };
		}

		public double RadiusEpoxyPhase { get; set; } = 0.2;

		public double ThicknessSilverPhase { get; set; } = 0.05;

		public void GeneratePhases(XModel<IXMultiphaseElement> physicalModel)
		{
			GeometryModel = new PhaseGeometryModel(physicalModel);
			physicalModel.GeometryModel = GeometryModel;
			GeometryModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(GeometryModel);
			GeometryModel.MergeOverlappingPhases = true;
			var defaultPhase = new DefaultPhase();
			GeometryModel.Phases[defaultPhase.ID] = defaultPhase;
			MatrixPhaseID = 0;

			double margin = ThicknessSilverPhase;
			var minCoordsExtended = new double[2];
			minCoordsExtended[0] = MinCoordinates[0] - margin;
			minCoordsExtended[1] = MinCoordinates[1] - margin;
			var maxCoordsExtended = new double[2];
			maxCoordsExtended[0] = MaxCoordinates[0] + margin;
			maxCoordsExtended[1] = MaxCoordinates[1] + margin;

			var ballsInternal = new List<Circle2D>();
			var ballsExternal = new List<Circle2D>();

			int b = 0;
			var rng = new Random(RngSeed);
			while (b < NumBalls)
			{
				var newCenter = new double[2];
				newCenter[0] = rng.NextDouble() * (maxCoordsExtended[0] - minCoordsExtended[0]) + minCoordsExtended[0];
				newCenter[1] = rng.NextDouble() * (maxCoordsExtended[1] - minCoordsExtended[1]) + minCoordsExtended[1];
				var newBallInternal = new Circle2D(newCenter, RadiusEpoxyPhase);
				var newBallExternal = new Circle2D(newCenter, RadiusEpoxyPhase + ThicknessSilverPhase);
				
				if (CollidesWithOtherBalls(newBallInternal, newBallExternal, ballsInternal, ballsExternal)) continue;
				ballsInternal.Add(newBallInternal);
				ballsExternal.Add(newBallExternal);

				// Create phases
				var phaseInternal = new LsmPhase(GeometryModel.Phases.Count, GeometryModel, -1);
				GeometryModel.Phases[phaseInternal.ID] = phaseInternal;
				EpoxyPhaseIDs.Add(phaseInternal.ID);
				var phaseExternal = new HollowLsmPhase(GeometryModel.Phases.Count, GeometryModel, 0);
				GeometryModel.Phases[phaseExternal.ID] = phaseExternal;
				SilverPhaseIDs.Add(phaseExternal.ID);

				// Create phase boundaries
				var lsmExternal = new SimpleLsm2D(phaseExternal.ID, physicalModel.Nodes.Values.OrderBy(n => n.ID).ToList(), 
					newBallExternal);
				var boundaryExternal = new ClosedPhaseBoundary(phaseExternal.ID, lsmExternal, defaultPhase, phaseExternal);
				defaultPhase.ExternalBoundaries.Add(boundaryExternal);
				defaultPhase.Neighbors.Add(phaseExternal);
				phaseExternal.ExternalBoundaries.Add(boundaryExternal);
				phaseExternal.Neighbors.Add(defaultPhase);

				var lsmInternal = new SimpleLsm2D(phaseInternal.ID, physicalModel.Nodes.Values.OrderBy(n => n.ID).ToList(), 
					newBallInternal);
				var boundaryInternal = new ClosedPhaseBoundary(phaseInternal.ID, lsmInternal, phaseExternal, phaseInternal);
				phaseExternal.InternalBoundaries.Add(boundaryInternal);
				phaseExternal.Neighbors.Add(phaseInternal);
				phaseExternal.InternalPhases.Add(phaseInternal);
				phaseInternal.ExternalBoundaries.Add(boundaryInternal);
				phaseInternal.Neighbors.Add(phaseExternal);

				++b;
			}

		}

		private bool CollidesWithOtherBalls(Circle2D newBallInternal, Circle2D newBallExternal, 
			List<Circle2D> ballsInternal, List<Circle2D> ballsExternal)
		{
			for (int i = 0; i < ballsInternal.Count; ++i)
			{
				double centerDistance = XFEM.Geometry.Utilities.Distance2D(newBallInternal.Center, ballsInternal[i].Center);
				if (newBallExternal.Radius + ballsInternal[i].Radius >= centerDistance) return true;
				if (newBallInternal.Radius + ballsExternal[i].Radius >= centerDistance) return true;
			}
			return false;
		}
	}
}
