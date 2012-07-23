using System;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Ccr.Core;
using Microsoft.Dss.Core.DsspHttp;
using Microsoft.Dss.Core.DsspHttpUtilities;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using W3C.Soap;


namespace UMass.LPR.Biclops
{
	public sealed class Contract
	{
		[DataMember]
		public const string Identifier = "http://schemas.tempuri.org/2009/03/biclops.html";
	}

    [DataContract]
    public enum MotorMode
    {
        OFFLINE = 0,
        INITIALIZED,
        HOMING,
        POSITION,
        PARKED,
    }; 

    [DataContract]
    enum JointIndexes {PAN, TILT};

	[DataContract]
	public class BiclopsState
	{
        [Description("Motor Mode")]
        [DataMember]
        public MotorMode Mode { get; set; }

        [Description("Joint Positions")]
        [DataMember]
        public List<double> JointPositions { get; set; }

        [Description("Joint Position References")]
        [DataMember]
        public List<double> JointPositionReferences { get; set; }
    }
	
	[ServicePort]
	public class BiclopsOperations : PortSet<DsspDefaultLookup, DsspDefaultDrop, Get, HttpGet, 
        SetJointPositionReference,
        Home, JointPositionUpdate, Subscribe, Park, UpdateState, HttpPost, QueryJointPositions,
        QueryDebug>
	{}
	
	public class Get : Get<GetRequestType, PortSet<BiclopsState, Fault>>
	{
		public Get()
		{
		}
		
		public Get(GetRequestType body)
			: base(body)
		{
		}
		
		public Get(GetRequestType body, PortSet<BiclopsState, Fault> responsePort)
			: base(body, responsePort)
		{
		}
	}

    public class Subscribe : Subscribe<SubscribeRequestType, PortSet<SubscribeResponseType, Fault>> {}

    //public class HttpGet : HttpGet<HttpGetRequestType, PortSet<HttpGetResponseType, Fault>> { } 

    // messages:
    [DataContract]
    public class SetJointPositionReferenceRequest
    {
        [DataMember]
        [Description("whether each joint should be set")]
        public List<bool> SetJoint { get; set; } // indicates whether each joint should be set (permits setting only one)

        [DataMember]
        [Description("Position References")]
        public List<double> PositionReference { get; set; }

        public SetJointPositionReferenceRequest() {
            // this constructor isn't being called.  I have no idea why.
            //Console.WriteLine("In SetJointPositionReferenceRequest constructor");
            // default to setting both
            SetJoint = new List<bool>();
            SetJoint.Add(true);
            SetJoint.Add(true);
            PositionReference = new List<double>();
            PositionReference.Add(0.0);
            PositionReference.Add(0.0);
        }
    }
    public class SetJointPositionReference : 
        Update<SetJointPositionReferenceRequest, PortSet<DefaultUpdateResponseType, Fault>>
    { }

    // home
    [DataContract]
    public class HomeRequest
    {
        public HomeRequest() { }
        // no member data needed
    }
    public class Home : Update<HomeRequest, PortSet<DefaultUpdateResponseType, Fault>> { }

    // Park
    [DataContract]
    public class ParkRequest
    {
        public ParkRequest() { }
        // no member data needed
    }
    public class Park : Update<ParkRequest, PortSet<DefaultUpdateResponseType, Fault>> { }

    // Update
    [DataContract]
    public class UpdateStateRequest
    {
        public UpdateStateRequest() { }
        // no member data needed
    }
    public class UpdateState : Update<UpdateStateRequest, PortSet<DefaultUpdateResponseType, Fault>> { }


    // notifications:
    // joint position update
    [DataContract]
    public class JointPositionUpdateRequest
    {
        [DataMember, DataMemberConstructor]
        [Description("Joint Positions")]
        public List<double> JointPositions { get; set; }

        public JointPositionUpdateRequest() {
            JointPositions = new List<double>();
        }
    }
    public class JointPositionUpdate : 
        Update<JointPositionUpdateRequest, PortSet<DefaultUpdateResponseType, Fault>> { }


    // queries
    [DataContract]
    public class QueryJointPositionsRequest{}

    [DataContract]
    public class QueryJointPositionsResponse
    {
        public QueryJointPositionsResponse()
        {
            JointPositions = new List<double>();
        }

        [Description("Joint Positions")]
        [DataMember]
        public List<double> JointPositions { get; set; }
    }
    public class QueryJointPositions : Query<QueryJointPositionsRequest, PortSet<QueryJointPositionsResponse, Fault>> { }


    // debuggery
    [DataContract]
    public class QueryDebugPayload { }

    [DataContract]
    public class QueryDebugResponse { }

    public class QueryDebug : Query<QueryDebugPayload, PortSet<QueryDebugResponse, Fault>> { }

}


