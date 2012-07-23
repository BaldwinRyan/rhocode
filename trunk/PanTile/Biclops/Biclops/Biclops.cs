using System;
using System.Net;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Ccr.Core;
using Microsoft.Dss.Core.DsspHttp;
using Microsoft.Dss.Core.DsspHttpUtilities;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using W3C.Soap;
using submgr = Microsoft.Dss.Services.SubscriptionManager;

using BiclopsAccessManaged;
using System.IO;
using System.Reflection;
using System.Collections.Specialized;

namespace UMass.LPR.Biclops
{
	[Contract(Contract.Identifier)]
	[DisplayName("Biclops")]
	[Description("Biclops service (no description provided)")]
	class BiclopsService : DsspServiceBase
	{
        [ServiceState]
        BiclopsState _state = null;
		
		[ServicePort("/Biclops", AllowMultipleInstances = false)]
		BiclopsOperations _mainPort = new BiclopsOperations();

        // Port to wait for state updates
        private Port<DateTime> _waitPort = new Port<DateTime>();

        // Declare and create a Subscription Manager to handle the subscriptions
        [SubscriptionManagerPartner]
        private submgr.SubscriptionManagerPort _submgrPort = new submgr.SubscriptionManagerPort();

        // Instance wrapper
        BiclopsAccessWrapper _biclops;

        [EmbeddedResource("UMass.LPR.Biclops.xslt")]
        string _transform = null;

        DsspHttpUtilitiesPort _httpUtilities;

		public BiclopsService(DsspServiceCreationPort creationPort)
			: base(creationPort)
		{

		}
		
		protected override void Start()
		{
            _biclops = new BiclopsAccessWrapper();

            if (_state == null)
            {
                _state = new BiclopsState();

                _state.Mode = (MotorMode)_biclops.getMode();

                _state.JointPositions = new List<double>();
                _state.JointPositionReferences = new List<double>();

                _state.JointPositions.Add(_biclops.getJointPosition(0));
                _state.JointPositions.Add(_biclops.getJointPosition(1));

                _state.JointPositionReferences.Add(_biclops.getDesiredJointPosition(0));
                _state.JointPositionReferences.Add(_biclops.getDesiredJointPosition(1));
            }

			base.Start();

            // Create utilities port
            _httpUtilities = DsspHttpUtilitiesService.Create(Environment);

            // start main update loop
            _waitPort.Post(DateTime.Now);
            Activate(Arbiter.Receive(true, _waitPort, UpdateLoopHandler));
        }

        // Update loop handler
        void UpdateLoopHandler(DateTime signal)
        {
  
           _mainPort.Post(new UpdateState());

            Activate(
                Arbiter.Receive(false, TimeoutPort(100),
                    delegate(DateTime time)
                    {
                        _waitPort.Post(time);
                    }
                )
            );

        }

        #region Service Handlers
        // [ServiceState] attribute obviates need for explicit handlers for Get, HttpGet, etc?

        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> SubscribeHandler(Subscribe subscribe)
        {
            SubscribeRequestType request = subscribe.Body;
            LogInfo("Subscribe request from: " + request.Subscriber);
            SubscribeHelper(_submgrPort, request, subscribe.ResponsePort);
            yield break;
        }

        #endregion

        #region Operations Handlers
        // SetJointPositionReference
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)] // do no use exclusive, otherwise it would block
        //[ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> SetJointPositionReferenceHandler(SetJointPositionReference update)
        {
            for (int i = 0; i < _state.JointPositionReferences.Count; i++)
            {
                if (update.Body.SetJoint[i])
                    _state.JointPositionReferences[i] = update.Body.PositionReference[i];
            }
            // send new references to driver layers...
            //Console.WriteLine("Biclops: set joint positions {0} {1}", _state.JointPositionReferences[0], _state.JointPositionReferences[1]);
            _biclops.setJointPosition(0, _state.JointPositionReferences[0]); 
            _biclops.setJointPosition(1, _state.JointPositionReferences[1]);

            update.ResponsePort.Post(new DefaultUpdateResponseType());
            yield break;
        }

        // Home
        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> HomeHandler(Home update)
        {
            _biclops.home();
            update.ResponsePort.Post(new DefaultUpdateResponseType());
            yield break;
        }

        // Park
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> ParkHandler(Park update)
        {
            _biclops.park();
            update.ResponsePort.Post(new DefaultUpdateResponseType());
            yield break;
        }

        // UpdateState
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)] //do no use exclusive, otherwise it would block
        public IEnumerator<ITask> UpdateStateHandler(UpdateState update)
        {
            _state.Mode = (MotorMode)_biclops.getMode();

            _state.JointPositions[0] = _biclops.getJointPosition(0);
            _state.JointPositions[1] = _biclops.getJointPosition(1);

            _state.JointPositionReferences[0] = _biclops.getDesiredJointPosition(0);
            _state.JointPositionReferences[1] = _biclops.getDesiredJointPosition(1);

            // Populate and send off JointPositionUpdate to all subscribing services.
            // This sends four positions: two joint positions and two reference positions.
            JointPositionUpdate upd = new JointPositionUpdate();
            upd.Body.JointPositions.Add(_state.JointPositions[0]);
            upd.Body.JointPositions.Add(_state.JointPositions[1]);
            upd.Body.JointPositions.Add(_state.JointPositionReferences[0]);
            upd.Body.JointPositions.Add(_state.JointPositionReferences[1]);

            base.SendNotification(_submgrPort, upd);

            //Console.WriteLine("Biclops update state handler: act: {0} {1} des: {2} {3}", 
            //    _state.JointPositions[0], _state.JointPositions[1], 
            //    _state.JointPositionReferences[0], _state.JointPositionReferences[1]);

            yield break;
        }

        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> JointPositionsQueryHandler(QueryJointPositions query)
        {
            //Console.WriteLine("Biclops: answering joint position query");
            QueryJointPositionsResponse jps = new QueryJointPositionsResponse();
            jps.JointPositions = _state.JointPositions;
            query.ResponsePort.Post(jps);
            yield break;
        }

        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> QueryDebugHandler(QueryDebug debug)
        {
            Console.WriteLine("Biclops: in QueryDebugHandler");
            debug.ResponsePort.Post(new QueryDebugResponse());
            yield break;
        }

        #endregion

        #region HTTP Handlers
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public virtual IEnumerator<ITask> HttpGetHandler(HttpGet httpGet)
        {
            httpGet.ResponsePort.Post(new HttpResponseType(HttpStatusCode.OK,
                _state,
                _transform));
            yield break;
        }

        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public virtual IEnumerator<ITask> HttpPostHandler(HttpPost httpPost)
        {
            Fault fault = null;
            NameValueCollection parameters = new NameValueCollection();

            ReadFormData readForm = new ReadFormData(httpPost);
            _httpUtilities.Post(readForm);

            // Wait for result
            yield return Arbiter.Choice(
                readForm.ResultPort,
                delegate(NameValueCollection col)
                {
                    parameters = col;
                },
                delegate(Exception e)
                {
                    fault = Fault.FromException(e);
                }
            );

            if(fault != null)
            {
                httpPost.ResponsePort.Post(fault);
                yield break;
            }

            if (!string.IsNullOrEmpty(parameters["NewMode"]))
            {
                int mod = int.Parse(parameters["NewMode"]);
                _biclops.setMode(mod);
            }

            if(!string.IsNullOrEmpty(parameters["pos1"]))
            {
                double newr = double.Parse(parameters["pos1"]);
                _biclops.setJointPosition(0, newr);
            }

            if(!string.IsNullOrEmpty(parameters["pos2"]))
            {
                double newr = double.Parse(parameters["pos2"]);
                _biclops.setJointPosition(1, newr);
            }

            HttpResponseType rsp = new HttpResponseType(HttpStatusCode.OK, _state, _transform);
            httpPost.ResponsePort.Post(rsp);

            SaveState(_state);
        }

        #endregion
    }
}


