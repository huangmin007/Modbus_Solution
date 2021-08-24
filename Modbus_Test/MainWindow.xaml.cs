using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using Modbus.Device;
using Modbus.Message;
using Modbus.Data;
using System.Collections.Generic;
using System.Globalization;

namespace Modbus_Test
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        private SerialPort serialPort;
        private IModbusMaster master;

        private Thread _thread;
        private ModbusSlave slave;
        private DataStore _dataStore = DataStoreFactory.CreateDefaultDataStore();

        private bool Running = true;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            serialPort = new SerialPort("COM27", 9600);
            serialPort.ReceivedBytesThreshold = 1;
            serialPort.Open();
            //master = ModbusSerialMaster.CreateRtu(serialPort);


            //slave = ModbusSerialSlave.CreateRtu(0, serialPort);
            //slave.ModbusSlaveRequestReceived += Slave_ModbusSlaveRequestReceived;
            //slave.WriteComplete += Slave_WriteComplete;
            //slave.DataStore = _dataStore;
            //slave.Listen();
            //slave.ListenAsync().GetAwaiter().GetResult();

            //_thread = new Thread(slave.Listen);// { Name = _port.ToString() };
            //_thread.Start();

#if true
            serialPort.DataReceived += SerialPort_DataReceived;
#else
            //ThreadPool.QueueUserWorkItem(new WaitCallback(ThreadFunction), this);
#endif
        }

        private void Slave_WriteComplete(object sender, ModbusSlaveRequestEventArgs e)
        {
            Console.WriteLine("WriteComplete....:{0}", e.Message);
        }

        private void Slave_ModbusSlaveRequestReceived(object sender, ModbusSlaveRequestEventArgs e)
        {            
            Console.WriteLine("requestReceived::,...{0}", e.Message);
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            byte[] buffer = new byte[serialPort.BytesToRead];
            int  len = serialPort.Read(buffer, 0, serialPort.BytesToRead);

            //Console.Write("HEX:{0}>>", len);
            //for(var i = 0; i < buffer.Length; i ++)
            //    Console.Write("{0:X2} ", buffer[i]);
            //Console.WriteLine();

            List<byte> buf = new List<byte>(buffer);

            while(buf.Count > 3)
            {
                IModbusMessage message = CreateModbusMessage(buf.ToArray());
                if (message == null) break;

                int length = message.MessageFrame.Length + 2;
                Console.Write("{0}>>", length);
                for (var i = 0; i < length; i++)
                    Console.Write("{0:X2} ", buf[i]);
                Console.WriteLine();
                Console.WriteLine(message);
                Console.WriteLine("SlaveAddress:{0}, FunctionCode:{1}", message.SlaveAddress, message.FunctionCode);

                if (message is IModbusRequest)  //IModbusMessage Request
                {
                    
                }
                else    //IModbusMessage Response
                {                    
                    ReadCoilsInputsResponse response = (ReadCoilsInputsResponse)message;

                    bool[] values = new bool[response.Data.Count];
                    response.Data.CopyTo(values, 0);
                }

                buf.RemoveRange(0, length);
            }

            buf.Clear();
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            Running = false;
            serialPort.Dispose();
            master.Dispose();

            base.OnClosing(e);
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            Button button = (Button)sender;
            if(button == Button_0)
            {
                master.WriteMultipleCoils(1, 0x0000, new bool[8] {false, false, false, false, false, false, false, false });
            }
            else if(button == Button_1)
            {
                master.WriteMultipleCoils(1, 0x0000, new bool[8] { true, true, true, true, true, true, true, true});
            }
        }

        protected void UpdateInputs(bool[] status)
        {
            if (status == null) return;
            Ellipse[] ells = { Ellipse_0, Ellipse_1, Ellipse_2, Ellipse_3, Ellipse_4, Ellipse_5, Ellipse_6, Ellipse_7  };

            int len = Math.Min(ells.Length, status.Length);

            this.Dispatcher.BeginInvoke((Action)delegate ()
                {
                    for (int i = 0; i < len; i++)
                    {
                        ells[i].Fill = status[i] ? Brushes.Red : Brushes.Gray;
                    }
                }
                );
        }

        protected void UpdateOutputs(bool[] status)
        {
            if (status == null) return;
            CheckBox[] boies = { CheckBox_0, CheckBox_1, CheckBox_2, CheckBox_3, CheckBox_4, CheckBox_5, CheckBox_6, CheckBox_7 };
            Rectangle[] rects = { Rectangle_0, Rectangle_1, Rectangle_2, Rectangle_3, Rectangle_4, Rectangle_5, Rectangle_6, Rectangle_7 };

            int len = Math.Min(rects.Length, status.Length);

            this.Dispatcher.BeginInvoke((Action)delegate ()
            {
                for (int i = 0; i < len; i++)
                {
                    boies[i].IsChecked = status[i];
                    rects[i].Fill = status[i] ? Brushes.Red : Brushes.Gray;
                }
            }                           
            );
        }

        private static void ThreadFunction(object obj)
        {
            MainWindow main = (MainWindow)obj;
            while(main.Running)
            {
                if(!main.serialPort.IsOpen)
                {
                    main.Running = false;
                    break;
                }

                bool[] inputs = null, outputs = null;

                try
                {
                    inputs = main.master.ReadInputs(1, 0x0000, 8);
                    outputs = main.master.ReadCoils(1, 0x0000, 8);
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.Message);
                    Thread.Sleep(1000);
                }
                
                main.UpdateInputs(inputs);
                main.UpdateOutputs(outputs);

                Thread.Sleep(500);
            }
        }

        private void CheckBox_Click(object sender, RoutedEventArgs e)
        {
            ushort address;
            CheckBox checkBox = (CheckBox)sender;

            if (!ushort.TryParse(checkBox.Name.Replace("CheckBox_", ""), out address))  return;

            try
            {
                master.WriteSingleCoil(1, address, checkBox.IsChecked.Value);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        public static IModbusMessage CreateModbusMessage(byte[] frame)
        {
            if (frame.Length < 3)
                throw new FormatException(string.Format("Argument 'frame' must have a length of at least {0} bytes.", 3));

            IModbusMessage message = null;

            byte functionCode = frame[1];
            switch (frame[1])
            {
                case 1:
                case 2:
                    try
                    {
                        message = ModbusMessageFactory.CreateModbusMessage<ReadCoilsInputsRequest>(frame);
                    }
                    catch (Exception ex)
                    {
                        message = ModbusMessageFactory.CreateModbusMessage<ReadCoilsInputsResponse>(frame);
                    }
                    return message;
                case 3:
                case 4:
                    try
                    {
                        message = ModbusMessageFactory.CreateModbusMessage<ReadHoldingInputRegistersRequest>(frame);
                    }
                    catch(Exception ex)
                    {
                        message = ModbusMessageFactory.CreateModbusMessage<ReadHoldingInputRegistersResponse>(frame);
                    }
                    return message;
                case 5:
                    return ModbusMessageFactory.CreateModbusMessage<WriteSingleCoilRequestResponse>(frame);
                case 6:
                    return ModbusMessageFactory.CreateModbusMessage<WriteSingleRegisterRequestResponse>(frame);
                case 7:
                case 8:
                    //return ModbusMessageFactory.CreateModbusMessage<DiagnosticsRequestResponse>(frame);
                case 9:
                case 10:
                case 11:
                case 12:
                case 13:
                case 14:
                    break;
                
                case 15:
                    try
                    {
                        message = ModbusMessageFactory.CreateModbusMessage<WriteMultipleCoilsRequest>(frame);
                    }
                    catch(Exception ex)
                    {
                        message = ModbusMessageFactory.CreateModbusMessage<WriteMultipleCoilsResponse>(frame);
                    }
                    return message; //ModbusMessageFactory.CreateModbusMessage<WriteMultipleRegistersRequest>(frame);
                default:
                    if (frame[1] == 23)
                        return ModbusMessageFactory.CreateModbusMessage<ReadWriteMultipleRegistersRequest>(frame);
                    break;
            }

            throw new ArgumentException(string.Format("Unsupported function code {0}", frame[1], "frame"));
        }
        
    }
}
