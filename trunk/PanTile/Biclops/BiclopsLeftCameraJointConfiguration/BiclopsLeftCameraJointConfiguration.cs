/*
 * 
 * Note: SOU (June/2009)
 * 
 * The BiclopsLeftJointConfiguration Service connects to the Biclops service
 * for getting and setting the joint information to the actual hardware/simulated hardware
 * 
 * The biclops service needs to running before you run this BiclopsJointConfiguration service
 * and the corresponding hostname/port needs to be added to the hosts.txt file
 * 
 * Tip: you can keep the biclops service running and code/compile/debug biclopsjointconfiguration
 *      this way you don't need to home everytime you make some changes
 * 
 */

using System;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Ccr.Core;
using Microsoft.Dss.Core;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.Core.DsspHttp;
using Microsoft.Dss.Core.DsspHttpUtilities;
using System.Net;
using System.IO;
using System.Threading;
using Microsoft.Dss.ServiceModel.Dssp;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using W3C.Soap;
using System.Collections.Specialized;
using MatrixLibrary;

using jointconfig = Robotics.JointConfiguration.Proxy;
using biclops = UMass.LPR.Biclops.Proxy;
using ds = Microsoft.Dss.Services.Directory;

namespace UMass.LPR.BiclopsLeftCameraJointConfiguration
{
	[Contract(Contract.Identifier)]
    [AlternateContract(jointconfig.Contract.Identifier)]
    [DisplayName("BiclopsLeftCameraJointConfiguration")]
	[Description("BiclopsLeftCameraJointConfiguration service")]
	class BiclopsLeftCameraJointConfigurationService : DsspServiceBase
	{
        /// <summary>
        /// _state
        /// </summary>
        [InitialStatePartner(Optional = true, ServiceUri = ServicePaths.Store + "\\biclopsleftcamerajointconfiguration.xml")]
        private jointconfig.JointConfigurationState _state;
        bool newDesiredJointPositions = true; // track whether the desired position has changed

        /// <summary>umas
        /// _main Port
        /// <summary>
        /// </summary>
        [ServicePort("/BiclopsLeftCameraJointConfiguration", AllowMultipleInstances = false)]
        private jointconfig.JointConfigurationOperations _mainPort = new jointconfig.JointConfigurationOperations();

        // port to wait for state updates - dunno whether we'll need something like this
        private Port<DateTime> _waitPort = new Port<DateTime>();

        // I think we'll need to write/copy this xslt thing:
        [EmbeddedResource("UMass.LPR.BiclopsLeftCameraJointConfiguration.BiclopsLeftCameraJointConfiguration.xslt")]
        string _transform = null;

        // http util port for reading from http page
        DsspHttpUtilitiesPort _httpUtilities;

        // port for Biclops brick (change this to work with simulated biclops)
        biclops.BiclopsOperations _biclopsPort = new UMass.LPR.Biclops.Proxy.BiclopsOperations();

		public BiclopsLeftCameraJointConfigurationService(DsspServiceCreationPort creationPort)
			: base(creationPort)
		{
		}

        // note that here the param order of the function matches the convention in the list
        // not so in uBotJointConfig
        void AddDHParameter(double alpha, double a, double d, double theta, string type)
        {
            _state.DHParams.Add(new List<double>(4) { alpha, a, d, theta });
            _state.LinkTypes.Add(type);
        }

		protected override void Start()
		{
			// needed for HttpPost
            _httpUtilities = DsspHttpUtilitiesService.Create(Environment);

            if (_state == null)
            {
                //int c = 0;

                LogInfo("Creating new BiclopsLeftCameraJointConfiguration State...");

                // get state
                _state = new jointconfig.JointConfigurationState();

                _state.NumLinks = 8; // this'll need to get updated accordingly
                _state.NumJoints = 2;

                _state.Moveable = true;


                _state.JointAngles = new Vector(_state.NumJoints);
                _state.DesiredJointAngles = new Vector(_state.NumJoints);
                _state.MinLimits = new Vector(_state.NumJoints);
                _state.MaxLimits = new Vector(_state.NumJoints);
                _state.MaxDelta = new Vector(_state.NumJoints);

                _state.LinkTypes = new List<string>();
                _state.DHParams = new List<List<double>>();

                // DH params will of course be totally different...but we'll need to add them in a similar fashion.
                /*
                 0.000000  -0.01   0.412   0.000000    REVOLUTE
                -1.570796   0.00   0.000   0.000000    REVOLUTE
                 1.570796   0.00   0.005   0.000000    CONSTANT
                -1.570796   0.00   0.000   0.000000    CONSTANT
                 1.570796   0.00   0.000  -1.570796    CONSTANT
                -1.570796   0.00   0.000   0.000000    CONSTANT
                 0.000000   0.00   0.040   0.000000    CONSTANT
                 0.000000   -0.09   0.000   0.000000    CONSTANT
                */

                //0
                AddDHParameter(0, -0.01, 0.412, 0, "LINK_TYPE_REVOLUTE");
                //1
                AddDHParameter(-Math.PI / 2.0, 0, 0, 0, "LINK_TYPE_REVOLUTE");
                //2
                AddDHParameter(Math.PI / 2.0, 0, 0.005, 0, "LINK_TYPE_CONSTANT");
                //3
                AddDHParameter(-Math.PI / 2.0, 0, 0, 0, "LINK_TYPE_CONSTANT");
                //4
                AddDHParameter(Math.PI / 2.0, 0, 0, -Math.PI / 2.0, "LINK_TYPE_CONSTANT");
                //5
                AddDHParameter(-Math.PI / 2.0, 0, 0, 0, "LINK_TYPE_CONSTANT");
                //6
                AddDHParameter(0, 0, 0.04, 0, "LINK_TYPE_CONSTANT");
                //7
                AddDHParameter(0, -0.09, 0, 0, "LINK_TYPE_CONSTANT");

                // we'll need to fix these, too
                _state.MinLimits.dat[0] = -2.0;
                _state.MinLimits.dat[1] = -0.35;

                _state.MaxLimits.dat[0] = 2.0;
                _state.MaxLimits.dat[1] = 1.2;

                _state.RobotName = "BiclopsLeft";

                _state.Lock = true;

                //set up default  vals
                for (int i = 0; i < 2; i++)
                {
                    _state.JointAngles.dat[i] = 0;
                    _state.DesiredJointAngles.dat[i] = 0;
                    _state.MaxDelta.dat[i] = 2.0; 
                    // because we're sending joint position references to drivers that already have reasonable
                    // PD gains, we don't need to worry about max delta here (so 2 radians per cycle is fine)
                }

                for (int i = 0; i < _state.NumLinks; i++)
                {
                    Console.Write("DH[{0}]=({1:F2} {2:F2} {3:F2} {4:F2})\n", i,
                       _state.DHParams[i][0],
                       _state.DHParams[i][1],
                       _state.DHParams[i][2],
                       _state.DHParams[i][3]);
                }

                for (int i = 0; i < _state.NumJoints; i++)
                {
                    Console.Write("Range[{0}]=({1:F2} {2:F2})\n", i,
                       _state.MinLimits.dat[i],
                       _state.MaxLimits.dat[i]);
                }
                for (int i = 0; i < _state.NumJoints; i++)
                {
                    Console.Write("MaxDelta[{0}]={1:F2}\n", i,
                       _state.MaxDelta.dat[i]);
                }

                base.SaveState(_state);
            }

            _state.Moveable = true;
            _state.Lock = true;

			// 
			// Add service specific initialization here
			// 
            SpawnIterator(ConnectToBiclopsService);
			
			base.Start();

            Console.WriteLine("Starting Biclops Left Camera JointConfiguration Thread");
            // start main update loop
            _waitPort.Post(DateTime.Now);
            Activate(Arbiter.Receive(true, _waitPort, UpdateLoopHandler));
		}



        #region Service Handlers

        void UpdateLoopHandler(DateTime signal)
        {
            if (_biclopsConnected)
            {
                _mainPort.Post(new jointconfig.UpdateState());
            }

            Activate(
                Arbiter.Receive(false, TimeoutPort(100), // should be 20, changed for debug
                    delegate(DateTime time)
                    {
                        _waitPort.Post(time);
                    }
                )
            );

        }

        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> UpdateStateHandler(jointconfig.UpdateState update)
        {
            bool bFault = false;

            if (bFault)
                yield break;
            //Console.WriteLine("BJC:UpdateStateHandler: locked = {0}", _state.Lock);
            // check to see if joints are locked...
            if (_state.Lock == false)
            {
                Vector Vdes = _state.DesiredJointAngles;

                //Console.WriteLine("BJC:UpdateStateHandler: inside lock == false");
                // LogInfo(LogGroups.Console, "Biclops Unlocked");
                
                // send the command to the biclops
                biclops.SetJointPositionReference r = new biclops.SetJointPositionReference();
                
                // only send a SetJointPositionReferenceRequest if the desired position has changed.
                if (newDesiredJointPositions)
                {
                    //Console.WriteLine("BJC:UpdateStateHandler: set new joints:{0}, {1}",
                    //    Vdes.dat[0], Vdes.dat[1]);
                    biclops.SetJointPositionReferenceRequest req =
                        new biclops.SetJointPositionReferenceRequest();
                    req.PositionReference = new List<double>();
                    req.PositionReference.Add(Vdes.dat[0]);
                    req.PositionReference.Add(Vdes.dat[1]);
                    req.SetJoint = new List<bool>();
                    req.SetJoint.Add(true);
                    req.SetJoint.Add(true);
                    Activate(
                    Arbiter.Choice(
                        _biclopsPort.SetJointPositionReference(req),
                        delegate(DefaultUpdateResponseType response)
                        {
                            //Console.WriteLine("got response from SetJointPosition");
                        },
                        delegate(W3C.Soap.Fault failure)
                        {
                            LogError("Fault posting SetJointPositionReference");
                            //Console.WriteLine("Fault posting SetJointPositionReference");
                        }));

                    newDesiredJointPositions = false;
                }
                 
            }
            else
            {
                // do nothing!
                //Console.WriteLine("BJC:UpdateStateHandler: inside else");
            }

            //Console.WriteLine("BJC: querying positions");
            // update the current joint angle state
            biclops.QueryJointPositionsRequest jpq = new UMass.LPR.Biclops.Proxy.QueryJointPositionsRequest();
            Activate(
                Arbiter.Choice(
                    _biclopsPort.QueryJointPositions(jpq),
                    delegate(biclops.QueryJointPositionsResponse response)
                    {
                        //Console.WriteLine("BJC: updated joint positions: ");
                        //Console.WriteLine("BJC: _state.NumJoints: {0}", _state.NumJoints); // fails at this line
                        for (int i = 0; i < _state.NumJoints; i++)
                        {
                            //////////////////////////////////////////////////////////////////////////
                            //Note: biclops' pan/tilt angles are in revers to the bisight's
                            //      the DH parameters are modified from the bisight
                            //      there is some rotation that needs to be reversed accordingly
                            //      but a quick fix is implemented here to reverse the angles
                            _state.JointAngles.dat[i] = -response.JointPositions[i];
                        }
                    },
                    delegate(W3C.Soap.Fault failure)
                    {
                        LogError("Fault calling QueryJointPositions");
                    }));

            update.ResponsePort.Post(DefaultUpdateResponseType.Instance); // why are we posting a response?

            yield break;
        }

        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public virtual IEnumerator<ITask> SetLockHandler(jointconfig.SetLock request)
        {
            _state.Lock = request.Body.LockValue;
            Console.Write("Biclops setting Lock to {0}\n", _state.Lock);
            request.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            yield break;
        }

        /// <summary>
        /// Get Handler
        /// </summary>
        /// <param name="get"></param>
        /// <returns></returns>
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public virtual IEnumerator<ITask> GetHandler(jointconfig.Get get)
        {
            get.ResponsePort.Post(_state);
            yield break;
        }


        /// <summary>
        /// Replace Handler
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> ReplaceHandler(jointconfig.Replace replace)
        {
            _state = replace.Body;
            replace.ResponsePort.Post(DefaultReplaceResponseType.Instance);
            yield break;
        }

        /// <summary>
        /// Http Get Handler
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public virtual IEnumerator<ITask> HttpGetHandler(HttpGet httpGet)
        {
            httpGet.ResponsePort.Post(new HttpResponseType(HttpStatusCode.OK,
                _state,
                _transform));
            yield break;
        }

        /// <summary>
        /// GetJointConfiguration Query Handler
        /// </summary>
        /// <param name="query"></param>
        /// <returns></returns>
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> GetJointConfigurationHandler(jointconfig.GetJointConfiguration query)
        {
            if (_state.JointAngles == null)
            {
                LogError("BiclopsJointCongfiguration -- can't return JointAngles!!");
                query.ResponsePort.Post(new Fault());
            }
            else
            {
                query.ResponsePort.Post(_state.JointAngles);
            }
            yield break;
        }

        /// <summary>
        /// Set Joint Delta Handler
        /// </summary>
        /// <param name="request">SetJointDelta</param>
        /// <returns></returns>
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        //[ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public virtual IEnumerator<ITask> SetJointDeltaHandler(jointconfig.SetJointDelta request)
        {
            Console.WriteLine("BJC: in SetJointDeltaHandler");
            if (_state == null || (request.Body.JointDelta.m != _state.NumJoints))
            {
                request.ResponsePort.Post(new Fault());
            }
            else
            {
                // set the joint delta, and add it to cur for desired
                // worry about scaling when we send to the biclops
                Vector Vdes = _state.DesiredJointAngles;
                Vector Vcur = _state.JointAngles;
                Vector Vu = request.Body.JointDelta;

                Vcur.add(ref Vu, ref Vdes);

                Console.WriteLine("Vdes:");
                Vdes.display();
                Console.WriteLine("_state.DesiredJointAngles:");
                _state.DesiredJointAngles.display();

                newDesiredJointPositions = true;

            }
            request.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            yield break;
        }

        /// <summary>
        /// Http Post Handler
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public virtual IEnumerator<ITask> HttpPostHandler(HttpPost httpPost)
        {
            string ErrorMessage = String.Empty;
            Fault fault = null;
            NameValueCollection parameters = new NameValueCollection();

            // use helper to read from data
            ReadFormData readForm = new ReadFormData(httpPost);
            _httpUtilities.Post(readForm);

            //wait for result
            yield return Arbiter.Choice(
                readForm.ResultPort,
                delegate(NameValueCollection col)
                {
                    parameters = col;
                },
                delegate(Exception e)
                {
                    fault = Fault.FromException(e);
                    LogError(null, "Error Processing from data", fault);
                    ErrorMessage += e;
                }
            );

            if (fault != null)
            {
                httpPost.ResponsePort.Post(fault);
                yield break;
            }

            bool validDelta = false;
            double[] _delta = new double[2];

            if (!string.IsNullOrEmpty(parameters["delta"]))
            {
                try
                {
                    _delta[0] = double.Parse(parameters["delta"]);
                    _delta[1] = double.Parse(parameters["delta"]);
                    validDelta = true;
                }
                catch (Exception e)
                {
                    string msg = "Could not parse Desired MaxDelta ref value: " + e.Message;
                    LogError(msg);
                    ErrorMessage += msg;
                }
            }
            if (validDelta)
            {
                _state.MaxDelta.dat[0] = _delta[0];
                _state.MaxDelta.dat[1] = _delta[1];
            }


            double[] _ref = new double[2];
            bool validValues = true;


            if (!string.IsNullOrEmpty(parameters["DesPan"]))
            {
                try
                {
                    _ref[0] = double.Parse(parameters["DesPan"]);
                }
                catch (Exception e)
                {
                    string msg = "Could not parse Desired Pan ref value: " + e.Message;
                    LogError(msg);
                    ErrorMessage += msg;
                    validValues = validValues && false;
                }
            }
            if (!string.IsNullOrEmpty(parameters["DesTilt"]))
            {
                try
                {
                    _ref[1] = double.Parse(parameters["DesTilt"]);
                }
                catch (Exception e)
                {
                    string msg = "Could not parse Desired Tilt ref value: " + e.Message;
                    LogError(msg);
                    ErrorMessage += msg;
                    validValues = validValues && false;
                }
            }


            if (!string.IsNullOrEmpty(parameters["lock"]))
            {
                try
                {
                    _state.Lock = true;
                }
                catch (Exception e)
                {
                    string msg = "Could not parse Lock check: " + e.Message;
                    LogError(msg);
                    ErrorMessage += msg;
                }
            }
            else
            {
                Console.WriteLine("Lock set to false");
                _state.Lock = false;
                newDesiredJointPositions = true;
            }

            if (validValues)
            {
                // scale down here...
                _state.DesiredJointAngles.dat[0] = _ref[0];
                _state.DesiredJointAngles.dat[1] = _ref[1];
            }

            if (!string.IsNullOrEmpty(parameters["Home"]))
            {
                Console.WriteLine("BiclopsLeftCameraJointConfiguration::Home()");
                _biclopsPort.Post(new biclops.Home());
                _state.Lock = true;
                Activate(
                Arbiter.Choice(
                    _biclopsPort.Home(new biclops.HomeRequest()),
                    delegate(DefaultUpdateResponseType response)
                    {
                        LogError(LogGroups.Console, "got response from Bicops.Home");
                        //Console.WriteLine("got response from Biclops.Home");
                    },
                    delegate(W3C.Soap.Fault failure)
                    {
                        LogError(LogGroups.Console, "Fault posting Bicops.Home");
                        //Console.WriteLine("Fault posting SetJointPositionReference");
                    }));
            }
            if (!string.IsNullOrEmpty(parameters["Park"]))
            {
                Console.WriteLine("BiclopsLeftCameraJointConfiguration::Park()");
                _biclopsPort.Post(new biclops.Park());
            }


            if (ErrorMessage == string.Empty)
            {
                httpPost.ResponsePort.Post(new HttpResponseType(HttpStatusCode.OK, _state, _transform));
            }
            else
            {
                fault = Fault.FromCodeSubcodeReason(FaultCodes.Receiver,
                                                    DsspFaultCodes.OperationFailed,
                                                    ErrorMessage);
                httpPost.ResponsePort.Post(new Fault());
            }
            yield break;
        }



        #endregion

        bool _biclopsConnected = false;
        List<string> _remoteHosts = new List<string>();

        private void loadRemoteServiceDirectoryPorts()
        {
            if (_remoteHosts == null)
            {
                _remoteHosts = new List<string>();
            }
            string hostsFileName = @"C:\Microsoft Robotics Dev Studio 2008\store\hosts.txt";
            FileStream f = File.Open(hostsFileName, FileMode.Open, FileAccess.Read, FileShare.Read);
            StreamReader sr = new StreamReader(f);
            string input = null;
            while ((input = sr.ReadLine()) != null)
            {
                Console.WriteLine("found remote host in file={0}", input);
                _remoteHosts.Add(input);
            }

            sr.Close();
            f.Close();
        }

        public IEnumerator<ITask> ConnectToBiclopsService()
        {
            _biclopsConnected = false;

            loadRemoteServiceDirectoryPorts();

            ds.DirectoryPort remoteDir;
            int k;

            List<biclops.BiclopsOperations> PortList = new List<biclops.BiclopsOperations>();

            while (_biclopsConnected==false)
            {
                for (k = 0; k < _remoteHosts.Count; k++)
                {
                    remoteDir = ServiceForwarder<ds.DirectoryPort>(_remoteHosts[k] + "/directory");
                    ServiceInfoType ServiceInfo = new ServiceInfoType(biclops.Contract.Identifier);
                    ds.QueryRequestType ReqBody;
                    ds.Query Query;

                    ReqBody = new ds.QueryRequestType(ServiceInfo);
                    Query = new ds.Query(ReqBody);

                    // Post query
                    remoteDir.Post(Query);
                    yield return Arbiter.Choice(Query.ResponsePort,
                        delegate(ds.QueryResponseType success)
                        {
                            if (success.RecordList != null)
                            {
                                Console.Write("Received {0} biclops service responses from directory\n", success.RecordList.Length);
                                foreach (ServiceInfoType si in success.RecordList)
                                {
                                    _biclopsPort = base.ServiceForwarder<biclops.BiclopsOperations>(si.Service);
                                    _biclopsConnected = true;
                                    break;
                                }
                            }
                        },
                        delegate(Fault f)
                        {
                            LogError(LogGroups.Console, f.ToString());
                        }
                    );
                    if (_biclopsConnected)
                        break;
                    else
                    {
                        Console.WriteLine("BiclopsLeftCameraJointConfiguration: cannot to connect to the Biclops Service\nStart one now if it has not been started and add the host/port to the hosts.txt");
                        Thread.Sleep(2000);
                    }
                }


            }

            yield break;
        }

	}

    /// <summary>
    /// Exception for received faults
    /// </summary>
    public class FaultException : Exception
    {
        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="message">error message</param>
        public FaultException(String message)
            : base(message)
        {
        }
    }


}


