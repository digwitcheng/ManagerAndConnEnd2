using AGV_V1._0.Event;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class MessageFactory
    {
        public static BaseMessage Create(MessageType type, string rawMessage)
        {
            switch (type)
            {
                case MessageType.DisConnect:
                    return new DisConnMessage().Create(rawMessage);
                case MessageType.Arrived:
                    return new ArrivedMessage().Create(rawMessage);
                case MessageType.ReStart:
                    return new ReStartMessage().Create(rawMessage);
                case MessageType.Move:
                    return new MoveMessage().Create(rawMessage);
                case MessageType.Msg:
                    return new MsgMessage().Create(rawMessage);
                case MessageType.AgvFile:
                    return new AgvFileMessage().Create(rawMessage);
                case MessageType.MapFile:
                    return new MapFileMessage().Create(rawMessage);
                default:
                    return new ErrorMessage().Create(rawMessage);
            }
        }
    }
}
