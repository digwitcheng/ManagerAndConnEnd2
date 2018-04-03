using AGV_V1._0.NLog;
using AGVSocket.Network;
using AGVSocket.Network.MyException;
using AGVSocket.Network.Packet;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.AgvNetwork.Packet
{
    class PacketFactory
    {
        public static ReceiveBasePacket CreateReceivePacket(PacketType type, byte[] data)
        {
            try
            {
                switch (type)
                {
                    case PacketType.DoneReply:
                        return new DoneReplyPacket(data);
                    case PacketType.AgvInfo:
                        return new AgvInfoPacket(data);
                    case PacketType.AgvResponse:
                        return new AgvResponsePacket(data);
                    default:
                        // return new ErrorPacket(data);
                        throw new PacketException("factory", ExceptionCode.NonsupportType);
                }
            }
            catch (PacketException pe)
            {
                //Send(new SysResponsePacket(1,buffers[));
                // throw;
                if (pe.Code == ExceptionCode.CheckSumError)
                {
                    return new ErrorPacket(data);
                }
                else if (pe.Code == ExceptionCode.DataMiss && data.Length >= 7)
                {
                    return new ErrorPacket(data);
                }
                else
                {
                    throw;
                }
            }
            catch (Exception ex)
            {
                Logs.Error("未知错误:" + ex);
                throw;
                // return new ErrorPacket(data);
            }

        }
    }
}
