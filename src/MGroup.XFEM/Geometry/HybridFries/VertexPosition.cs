using System;
using System.Collections.Generic;
using System.Text;

//TODO: perhaps I should also have extension vertices here, instead of using another property in Vertex2D/3D
namespace MGroup.XFEM.Geometry.HybridFries
{
	public enum VertexPosition
	{
		/// <summary>
		/// Lies on the extension of the 2D/3D crack. 
		/// </summary>
		Extension,

		/// <summary>
		/// Regular vertex of the 2D/3D crack mesh. Does not belong to the crack front.
		/// </summary>
		Internal,

		/// <summary>
		/// Belongs to the crack front in a geometric sense. However it is not an actual crack tip: it does not affect the 
		/// displacement field around it and the crack cannot propagate from it. Examples: vertices of the crack front that lie
		/// on the boundary of the domain (crack "mouths")
		/// </summary>
		TipInactive, 
		
		/// <summary>
		/// Belongs to the crack front, affects the displacement field around it and the crack will propagate from it.
		/// </summary>
		TipActive
	}
}
