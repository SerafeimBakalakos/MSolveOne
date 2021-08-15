//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Linq;
//using System.Text;
//using MGroup.XFEM.Cracks.Geometry;
//using MGroup.XFEM.Enrichment.Functions;
//using MGroup.XFEM.Entities;

//namespace MGroup.XFEM.Enrichment.Observers
//{
//	public class RejectedCrackStepNodesObserver : IEnrichmentObserver_v2
//	{
//		private readonly ICrack crack;
//		private readonly NewCrackTipNodesObserver tipNodesObserver;
//		private readonly AllCrackStepNodesObserver stepNodesObserver;

//		public RejectedCrackStepNodesObserver(ICrack crack, NewCrackTipNodesObserver tipNodesObserver,
//			AllCrackStepNodesObserver stepNodesObserver)
//		{
//			this.crack = crack;
//			this.tipNodesObserver = tipNodesObserver;
//			this.stepNodesObserver = stepNodesObserver;
//		}

//		public HashSet<XNode> RejectedHeavisideNodes { get; } = new HashSet<XNode>();

//		public void EndCurrentAnalysisIteration()
//		{


//			WriteToDebug();
//		}

//		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
//		{
//		}

//		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
//		{
//		}

//		public void StartNewAnalysisIteration()
//		{
//			RejectedHeavisideNodes.Clear();
//		}

//		public void WriteToDebug()
//		{
//			var msg = new StringBuilder("Rejected crack step nodes:");
//			foreach (XNode node in RejectedHeavisideNodes.OrderBy(n => n.ID))
//			{
//				msg.Append(" " + node.ID);
//			}
//			Debug.WriteLine(msg);
//		}
//	}
//}
