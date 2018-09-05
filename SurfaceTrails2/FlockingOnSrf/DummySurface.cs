using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace SurfaceTrails2.FlockingOnSrf
{
    public class DummySurface : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the DummySurface class.
        /// </summary>
        public DummySurface()
          : base("DummySurface", "Nickname",
              "Description",
              "Category", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddSurfaceParameter("srf", "srf", "srf", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Positions", "Positions", "The agent positions", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Surface surface = null;
            DA.GetData("srf", ref surface);

            Point3d point;

            Interval dom =new Interval(0, 1);
            surface.SetDomain(0, dom);
            surface.SetDomain(1, dom);

            point = surface.PointAt(0.2, 0.8);

            var a = point;
            DA.SetData(0, a);


        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("7289387b-a6f3-4da2-bf61-25583b47ff72"); }
        }
    }
}