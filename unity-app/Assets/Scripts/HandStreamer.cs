using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using NativeWebSocket;
using Newtonsoft.Json;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;

namespace VitureTeleop
{
    /// <summary>
    /// Captures the right-hand 26-joint skeleton from Unity's XRHands subsystem
    /// (which Viture's Unity XR SDK integrates with) and streams each frame as
    /// JSON over a WebSocket to the retargeting service on the laptop.
    /// </summary>
    public class HandStreamer : MonoBehaviour
    {
        [Header("Connection")]
        [Tooltip("ws://<laptop-ip>:<port> ; ConfigPanel overrides at runtime.")]
        public string defaultUrl = "ws://192.168.1.100:8765";

        [Header("Streaming")]
        [Range(1f, 90f)] public float targetHz = 30f;
        public bool streamLeftHand = true;
        public bool streamRightHand = true;

        private WebSocket _socket;
        private XRHandSubsystem _handSubsystem;
        private float _nextSendTime;
        private long _seq;
        private string _activeUrl;

        // The 26 joint IDs in the canonical order shared with the Python service.
        private static readonly XRHandJointID[] JointOrder = new[]
        {
            XRHandJointID.Palm,
            XRHandJointID.Wrist,
            XRHandJointID.ThumbMetacarpal,
            XRHandJointID.ThumbProximal,
            XRHandJointID.ThumbDistal,
            XRHandJointID.ThumbTip,
            XRHandJointID.IndexMetacarpal,
            XRHandJointID.IndexProximal,
            XRHandJointID.IndexIntermediate,
            XRHandJointID.IndexDistal,
            XRHandJointID.IndexTip,
            XRHandJointID.MiddleMetacarpal,
            XRHandJointID.MiddleProximal,
            XRHandJointID.MiddleIntermediate,
            XRHandJointID.MiddleDistal,
            XRHandJointID.MiddleTip,
            XRHandJointID.RingMetacarpal,
            XRHandJointID.RingProximal,
            XRHandJointID.RingIntermediate,
            XRHandJointID.RingDistal,
            XRHandJointID.RingTip,
            XRHandJointID.LittleMetacarpal,
            XRHandJointID.LittleProximal,
            XRHandJointID.LittleIntermediate,
            XRHandJointID.LittleDistal,
            XRHandJointID.LittleTip,
        };

        private static readonly string[] JointIdStrings = new[]
        {
            "palm", "wrist",
            "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
            "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
            "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
            "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
            "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
        };

        private void Awake()
        {
            Debug.Log($"HandStreamer: Awake on GameObject '{gameObject.name}'");
        }

        private async void Start()
        {
            Debug.Log($"HandStreamer: Start; defaultUrl={defaultUrl}");
            _activeUrl = PlayerPrefs.GetString("viture_teleop.url", defaultUrl);
            Debug.Log($"HandStreamer: activeUrl={_activeUrl}");
            await Connect();
            _handSubsystem = GetHandSubsystem();
            if (_handSubsystem == null)
            {
                Debug.LogError("HandStreamer: no XRHandSubsystem found. Is Viture's Unity XR SDK installed and XR Plugin Management enabled?");
            }
            else
            {
                Debug.Log("HandStreamer: XRHandSubsystem found, tracking will start once hands are visible");
            }
        }

        private async Task Connect()
        {
            if (_connecting) return;
            _connecting = true;
            try
            {
                if (_socket != null)
                {
                    try { await _socket.Close(); } catch (Exception) { }
                    _socket = null;
                }
                _socket = new WebSocket(_activeUrl);
                _socket.OnOpen += () => Debug.Log($"HandStreamer: connected to {_activeUrl}");
                _socket.OnError += (e) => Debug.LogWarning($"HandStreamer: error {e}");
                _socket.OnClose += (c) => Debug.LogWarning($"HandStreamer: closed {c}");
                _ = _socket.Connect();
            }
            finally
            {
                _connecting = false;
            }
        }

        public async void SetUrl(string url)
        {
            _activeUrl = url;
            PlayerPrefs.SetString("viture_teleop.url", url);
            PlayerPrefs.Save();
            await Connect();
        }

        private static XRHandSubsystem GetHandSubsystem()
        {
            var xrManagerSettings = XRGeneralSettings.Instance?.Manager;
            if (xrManagerSettings == null) return null;
            var subsystems = new List<XRHandSubsystem>();
            SubsystemManager.GetSubsystems(subsystems);
            return subsystems.Count > 0 ? subsystems[0] : null;
        }

        private long _rightSent;
        private long _leftSent;
        private bool _wasRightTracked;
        private bool _wasLeftTracked;
        private bool _wasSocketOpen;
        private float _nextHeartbeat;
        // Auto-reconnect plumbing: if the socket ever drops out of Open
        // (server restarted, network blip, etc.), retry every 2 s without
        // requiring an APK rebuild.
        private float _nextReconnectTime;
        private const float ReconnectIntervalSeconds = 2f;
        private bool _connecting;

        private void Update()
        {
#if !UNITY_WEBGL || UNITY_EDITOR
            _socket?.DispatchMessageQueue();
#endif

            // Periodic heartbeat: prints once a second whether we're connected and what we're tracking.
            // This guarantees the log always shows *some* status even when nothing's flowing.
            bool socketOpen = _socket != null && _socket.State == WebSocketState.Open;
            if (Time.unscaledTime >= _nextHeartbeat)
            {
                _nextHeartbeat = Time.unscaledTime + 1f;
                bool rT = _handSubsystem != null && _handSubsystem.rightHand.isTracked;
                bool lT = _handSubsystem != null && _handSubsystem.leftHand.isTracked;
                Debug.Log(
                    $"HandStreamer: heartbeat socket={(socketOpen ? "OPEN" : (_socket == null ? "NULL" : _socket.State.ToString()))} "
                    + $"rightTracked={rT} leftTracked={lT} sentR={_rightSent} sentL={_leftSent}");
            }
            if (socketOpen != _wasSocketOpen)
            {
                Debug.Log($"HandStreamer: socket transition -> {(socketOpen ? "OPEN" : "CLOSED")}");
                _wasSocketOpen = socketOpen;
                // Reset the retry timer on transition so reconnects fire promptly.
                _nextReconnectTime = Time.unscaledTime;
            }

            // Auto-reconnect: if the socket isn't currently open or trying to
            // open, retry on a timer. Avoids needing an APK rebuild when the
            // laptop server restarts (or the network briefly drops).
            bool needsReconnect = _socket == null
                || _socket.State == WebSocketState.Closed
                || _socket.State == WebSocketState.Closing;
            if (needsReconnect && !_connecting && Time.unscaledTime >= _nextReconnectTime)
            {
                _nextReconnectTime = Time.unscaledTime + ReconnectIntervalSeconds;
                Debug.Log($"HandStreamer: attempting reconnect to {_activeUrl}");
                _ = Connect();
            }

            if (!socketOpen) return;
            if (_handSubsystem == null) return;

            float period = 1f / Mathf.Max(1f, targetHz);
            if (Time.unscaledTime < _nextSendTime) return;
            _nextSendTime = Time.unscaledTime + period;

            // Track tracked-state transitions independently for each hand.
            bool rNow = _handSubsystem.rightHand.isTracked;
            bool lNow = _handSubsystem.leftHand.isTracked;
            if (rNow != _wasRightTracked)
            {
                Debug.Log($"HandStreamer: right hand tracked={rNow}");
                _wasRightTracked = rNow;
            }
            if (lNow != _wasLeftTracked)
            {
                Debug.Log($"HandStreamer: left hand tracked={lNow}");
                _wasLeftTracked = lNow;
            }

            if (streamRightHand && rNow)
            {
                SendHand("right", _handSubsystem.rightHand);
                _rightSent++;
                if (_rightSent == 1) Debug.Log("HandStreamer: first RIGHT frame sent");
            }
            if (streamLeftHand && lNow)
            {
                SendHand("left", _handSubsystem.leftHand);
                _leftSent++;
                if (_leftSent == 1) Debug.Log("HandStreamer: first LEFT frame sent");
            }
        }

        // Map each XRHandJointID -> its proximal neighbor when we need to synthesize
        // a metacarpal. Viture's SDK reports only 21 joints (no metacarpals), so we
        // fabricate each metacarpal as a point between the wrist and the corresponding
        // proximal joint (roughly where the palm knuckle sits anatomically).
        private static readonly Dictionary<XRHandJointID, XRHandJointID> MetacarpalProximal = new()
        {
            { XRHandJointID.ThumbMetacarpal,  XRHandJointID.ThumbProximal },
            { XRHandJointID.IndexMetacarpal,  XRHandJointID.IndexProximal },
            { XRHandJointID.MiddleMetacarpal, XRHandJointID.MiddleProximal },
            { XRHandJointID.RingMetacarpal,   XRHandJointID.RingProximal },
            { XRHandJointID.LittleMetacarpal, XRHandJointID.LittleProximal },
        };
        private const float MetacarpalInterp = 0.25f;  // 25% from wrist toward proximal

        private void SendHand(string hand, XRHand xrHand)
        {
            var wristJoint = xrHand.GetJoint(XRHandJointID.Wrist);
            bool wristOk = wristJoint.TryGetPose(out var wristPose);

            var joints = new List<Dictionary<string, object>>(JointOrder.Length);
            bool tracked = wristOk;
            for (int i = 0; i < JointOrder.Length; i++)
            {
                var id = JointOrder[i];
                Pose pose;
                var joint = xrHand.GetJoint(id);
                if (!joint.TryGetPose(out pose))
                {
                    if (MetacarpalProximal.TryGetValue(id, out var proximalId)
                        && xrHand.GetJoint(proximalId).TryGetPose(out var proximalPose)
                        && wristOk)
                    {
                        pose = new Pose(
                            Vector3.Lerp(wristPose.position, proximalPose.position, MetacarpalInterp),
                            Quaternion.Slerp(wristPose.rotation, proximalPose.rotation, MetacarpalInterp)
                        );
                    }
                    else
                    {
                        tracked = false;
                        pose = default;
                    }
                }
                joints.Add(new Dictionary<string, object>
                {
                    { "id", JointIdStrings[i] },
                    { "p", new[] { pose.position.x, pose.position.y, pose.position.z } },
                    { "r", new[] { pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w } },
                });
            }

            var msg = new Dictionary<string, object>
            {
                { "t", (double)DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0 },
                { "seq", _seq++ },
                { "hand", hand },
                { "tracked", tracked },
                { "wrist_pos", new[] { wristPose.position.x, wristPose.position.y, wristPose.position.z } },
                { "wrist_rot", new[] { wristPose.rotation.x, wristPose.rotation.y, wristPose.rotation.z, wristPose.rotation.w } },
                { "joints", joints },
            };

            string json = JsonConvert.SerializeObject(msg);
            try
            {
                _socket.SendText(json);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"HandStreamer: SendText failed for {hand}: {e.GetType().Name} {e.Message}");
            }
        }

        private async void OnDestroy()
        {
            if (_socket != null)
            {
                try { await _socket.Close(); } catch (Exception) { }
            }
        }
    }
}
