namespace MGroup.XFEM.IsoXFEM.MeshGeneration
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.XFEM.Entities;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;

	public interface IMeshGeneration
	{
		public GeometryProperties GeometryOfModel { get; }
		public Dictionary<int, IIsoXfemElement> CreateElements(Dictionary<int, XNode> nodes);
		public Dictionary<int, XNode> CreateNodes();
		public (Dictionary<int, XNode> nodes, Dictionary<int, IIsoXfemElement> elements) MakeMesh();
	}
}
