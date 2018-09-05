using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace SurfaceTrails2.FlockingOnSrf
{
    public class FlockSystem
    {
        public List<FlockAgent> Agents;

        public double Timestep;
        public double NeighbourhoodRadius;
        public double AlignmentStrength;
        public double CohesionStrength;
        public double SeparationStrength;
        public double SeparationDistance;
        public List<Circle> Repellers;
        public bool UseParallel;
         NurbsSurface _surface;
        private double _boundingBoxSize = 1;
        private Random _randomX = new Random();
        private Random _randomY = new Random();


        public double BoundingBox { get { return _boundingBoxSize; } set { _boundingBoxSize = value; } }
        public NurbsSurface Surface { get { return _surface; } set { _surface = value; } }


        public FlockSystem(int agentCount/*, bool is3D*/,NurbsSurface surface)
        {
            Surface = surface;
            Agents = new List<FlockAgent>();
            Random random = new Random();
            Interval domainU = _surface.Domain(0);
            Interval domainV = _surface.Domain(1);

   


            _surface.SetDomain(0, domainU);
            _surface.SetDomain(0, domainV);

            double randomCoordinateX = _randomX.NextDouble();
            double randomCoordinateY = _randomX.NextDouble();

          

            var tu = domainU.ParameterAt(random.NextDouble());
            var tv = domainV.ParameterAt(random.NextDouble());
            //var vectorX = _surface.PointAt(Util.GetRandomUnitVectorX(), Util.GetRandomUnitVectorY()).X;
            //var vectory = _surface.PointAt(Util.GetRandomUnitVectorX(), Util.GetRandomUnitVectorY()).Y;

            var vectorX = _surface.PointAt(Util.GetRandomUnitVectorX(), Util.GetRandomUnitVectorY()).X;
            var vectory = _surface.PointAt(Util.GetRandomUnitVectorX(), Util.GetRandomUnitVectorY()).Y;


            //if (is3D)
            //    for (int i = 0; i < agentCount; i++)
            //    {
            //        FlockAgent agent = new FlockAgent(
            //            Util.GetRandomPoint(0.0, 30.0, 0.0, 30.0, 0.0, 30.0),
            //            Util.GetRandomUnitVector() * 4.0);

            //        agent.FlockSystem = this;

            //        Agents.Add(agent);
            //    }
            //else
            for (int i = 0; i < agentCount; i++)
                {
                    FlockAgent agent = new FlockAgent(
                        _surface.PointAt(tu, tv),new Vector3d(Point3d.Subtract(_surface.PointAt(tu, tv), 
                            _surface.PointAt(tu/(_surface.Domain(0).T1-_surface.Domain(0).T0),tv/ (_surface.Domain(1).T1 - _surface.Domain(0).T1)))) );


                agent.FlockSystem = this;

                    Agents.Add(agent);
                }
        }


        private List<FlockAgent> FindNeighbours(FlockAgent agent)
        {
            var neighbours = new List<FlockAgent>();

            foreach (FlockAgent neighbour in Agents)
                if (neighbour != agent && neighbour.Position.DistanceTo(agent.Position) < NeighbourhoodRadius)
                    neighbours.Add(neighbour);

            return neighbours;
        }

        private void ComputeAgentDesiredVelocity(FlockAgent agent)
        {
            List<FlockAgent> neighbours = FindNeighbours(agent);
            agent.ComputeDesiredVelocity(neighbours);
        }


        public void Update()
        {
            if (UseParallel)
                Parallel.ForEach(Agents, ComputeAgentDesiredVelocity);
            else
                foreach (var agent in Agents)
                    ComputeAgentDesiredVelocity(agent);

            // Once the desired velocity for each agent has been computed, we update each position and velocity
            foreach (var agent in Agents)
                agent.UpdateVelocityAndPosition();
        }


        public void UpdateUsingRTree()
        {
            /* First, build the R-Tree */

            var rTree = new RTree();

            for (var i = 0; i < Agents.Count; i++)
                rTree.Insert(Agents[i].Position, i);

            /* Then, we use the R-Tree to find the neighbours
                and compute the desired velocity */

            foreach (var agent in Agents)
            {
                List<FlockAgent> neighbours = new List<FlockAgent>();

                EventHandler<RTreeEventArgs> rTreeCallback =
                (object sender, RTreeEventArgs args) =>
                {
                    if (Agents[args.Id] != agent)
                        neighbours.Add(Agents[args.Id]);
                };

                rTree.Search(new Sphere(agent.Position, NeighbourhoodRadius), rTreeCallback);

                agent.ComputeDesiredVelocity(neighbours);
            }



            // Once the desired velocity for each agent has been computed, we can update each position and velocity               
            foreach (FlockAgent agent in Agents) agent.UpdateVelocityAndPosition();
        }
    }
}
