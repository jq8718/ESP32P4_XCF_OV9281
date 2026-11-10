using System;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.ComTypes;
[ComImport, Guid("29840822-5B84-11D0-BD3B-00A0C911CE86"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface ICreateDevEnum { [PreserveSig] int CreateClassEnumerator([In] ref Guid category,out IEnumMoniker list,int flags); }
[ComImport,Guid("56A86895-0AD4-11CE-B03A-0020AF0BA770"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface IBaseFilter {}
[ComImport, Guid("56A868A9-0AD4-11CE-B03A-0020AF0BA770"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface IGraphBuilder { [PreserveSig]int AddFilter([In]IBaseFilter filter,[In,MarshalAs(UnmanagedType.LPWStr)]string name); }
[ComImport,Guid("93E5A4E0-2D50-11d2-ABFA-00A0C9C6E38D"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface ICaptureGraphBuilder2 {
 [PreserveSig]int SetFiltergraph([In]IGraphBuilder graph);
 [PreserveSig]int GetFiltergraph(out IGraphBuilder graph);
 [PreserveSig]int SetOutputFileName(IntPtr type,string name,IntPtr a,IntPtr b);
 [PreserveSig]int FindInterface(IntPtr category,IntPtr type,object filter,ref Guid iid,out object o);
 [PreserveSig]int RenderStream([In]ref Guid category,[In]ref Guid type,[In,MarshalAs(UnmanagedType.IUnknown)]object source,[In]IBaseFilter intermediate,[In]IBaseFilter sink);
}
public class DshowCheck {
 public static void Run() {
 var de=(ICreateDevEnum)Activator.CreateInstance(Type.GetTypeFromCLSID(new Guid("62BE5D10-60EB-11d0-BD3B-00A0C911CE86")));
 Guid category=new Guid("860BB310-5D01-11D0-BD3B-00A0C911CE86"); IEnumMoniker en;de.CreateClassEnumerator(ref category,out en,0);
 IMoniker[] m=new IMoniker[1];
 while(en.Next(1,m,IntPtr.Zero)==0) {
 string name; m[0].GetDisplayName(null,null,out name); if(!name.Contains("vid_303a"))continue;
 Guid iid=new Guid("56A86895-0AD4-11CE-B03A-0020AF0BA770");object source;m[0].BindToObject(null,null,ref iid,out source);
 var graph=(IGraphBuilder)Activator.CreateInstance(Type.GetTypeFromCLSID(new Guid("E436EBB3-524F-11CE-9F53-0020AF0BA770")));
 var builder=(ICaptureGraphBuilder2)Activator.CreateInstance(Type.GetTypeFromCLSID(new Guid("BF87B6E1-8C27-11D0-B3F0-00AA003761C5")));
 builder.SetFiltergraph(graph); graph.AddFilter((IBaseFilter)source,"camera");
 object sink=Activator.CreateInstance(Type.GetTypeFromCLSID(new Guid("C1F400A4-3F08-11D3-9F0B-006008039E37")));graph.AddFilter((IBaseFilter)sink,"sink");
 Guid cap=new Guid("FB6C4281-0353-11D1-905F-0000C0CC16BA"),video=new Guid("73646976-0000-0010-8000-00AA00389B71");
 int hr=builder.RenderStream(ref cap,ref video,source,null,(IBaseFilter)sink);Console.WriteLine(name+" RenderStream="+hr.ToString("X8"));
 Marshal.ReleaseComObject(builder);Marshal.ReleaseComObject(graph);Marshal.ReleaseComObject(source);Marshal.ReleaseComObject(sink);
 }
 }
}
