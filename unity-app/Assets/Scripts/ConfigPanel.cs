using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace VitureTeleop
{
    /// <summary>
    /// Minimal in-app config: one text field for the laptop IP, one for port,
    /// and a Connect button. Wires up a HandStreamer on this GameObject.
    /// </summary>
    [RequireComponent(typeof(HandStreamer))]
    public class ConfigPanel : MonoBehaviour
    {
        public TMP_InputField ipField;
        public TMP_InputField portField;
        public Button connectButton;
        public TMP_Text statusText;

        private HandStreamer _streamer;
        private const string IpKey = "viture_teleop.ip";
        private const string PortKey = "viture_teleop.port";

        private void Start()
        {
            _streamer = GetComponent<HandStreamer>();
            if (ipField != null) ipField.text = PlayerPrefs.GetString(IpKey, "192.168.1.100");
            if (portField != null) portField.text = PlayerPrefs.GetString(PortKey, "8765");
            if (connectButton != null) connectButton.onClick.AddListener(OnConnect);
        }

        private void OnConnect()
        {
            string ip = ipField != null ? ipField.text : "192.168.1.100";
            string port = portField != null ? portField.text : "8765";
            PlayerPrefs.SetString(IpKey, ip);
            PlayerPrefs.SetString(PortKey, port);
            PlayerPrefs.Save();

            string url = $"ws://{ip}:{port}";
            _streamer.SetUrl(url);
            if (statusText != null) statusText.text = $"Connecting → {url}";
        }
    }
}
