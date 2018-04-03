using AGV_V1._0.Event;
using Cowboy.Sockets;
using System;

namespace AGV_V1._0.Network.Messages
{
    public abstract class BaseMessage
    {
        protected const int SUCCESS = 6;
        protected const int FAIRED = -1;
        public MessageType Type { get;  set; }
        public string Message { get; set; }
        public abstract BaseMessage Create(string msg);
        public abstract void Send(string sessionKey, TcpSocketServer _server);
        public abstract void Receive();

        public event EventHandler<MessageEventArgs> ShowMessage;
        public event EventHandler<MessageEventArgs> DataMessage;
        public delegate void ReLoadDele();
        public ReLoadDele ReLoad;

        
        protected void OnMessageEvent(string message)
        {
            try
            {
                if (null != ShowMessage)
                {
                    ShowMessage.Invoke(this, new MessageEventArgs(message));
                }
            }
            catch
            {

            }
        }
        protected void OnTransmitEvent(object sender, MessageEventArgs e)
        {
            try
            {
                if (null != DataMessage)
                {
                    DataMessage.Invoke(sender, e);
                }
            }
            catch
            {

            }
        }
        
    }
}
