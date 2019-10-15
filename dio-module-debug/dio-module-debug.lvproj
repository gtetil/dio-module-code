<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="19008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="dio-module-debug.vi" Type="VI" URL="../dio-module-debug.vi"/>
		<Item Name="NEO VI Example.vi" Type="VI" URL="../NEO VI Example.llb/NEO VI Example.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Trim Whitespace.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Trim Whitespace.vi"/>
				<Item Name="whitespace.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/whitespace.ctl"/>
			</Item>
			<Item Name="build status string.vi" Type="VI" URL="../NEO VI Example.llb/build status string.vi"/>
			<Item Name="closeneodevices.vi" Type="VI" URL="../NEO VI Example.llb/closeneodevices.vi"/>
			<Item Name="const_cluster_firesetting.vi" Type="VI" URL="../NEO VI Example.llb/const_cluster_firesetting.vi"/>
			<Item Name="const_cluster_spymessage.vi" Type="VI" URL="../NEO VI Example.llb/const_cluster_spymessage.vi"/>
			<Item Name="const_cluster_svcan3settings.vi" Type="VI" URL="../NEO VI Example.llb/const_cluster_svcan3settings.vi"/>
			<Item Name="ctl firesetting.ctl" Type="VI" URL="../NEO VI Example.llb/ctl firesetting.ctl"/>
			<Item Name="ctl networkid.ctl" Type="VI" URL="../NEO VI Example.llb/ctl networkid.ctl"/>
			<Item Name="ctl spy protocols.ctl" Type="VI" URL="../NEO VI Example.llb/ctl spy protocols.ctl"/>
			<Item Name="ctl spymessage.ctl" Type="VI" URL="../NEO VI Example.llb/ctl spymessage.ctl"/>
			<Item Name="ctl StatusBitfield.ctl" Type="VI" URL="../NEO VI Example.llb/ctl StatusBitfield.ctl"/>
			<Item Name="ctl svcansetting.ctl" Type="VI" URL="../NEO VI Example.llb/ctl svcansetting.ctl"/>
			<Item Name="findneodevices.vi" Type="VI" URL="../NEO VI Example.llb/findneodevices.vi"/>
			<Item Name="forumlate message write.vi" Type="VI" URL="../NEO VI Example.llb/forumlate message write.vi"/>
			<Item Name="getconfiguration.vi" Type="VI" URL="../NEO VI Example.llb/getconfiguration.vi"/>
			<Item Name="getdllversion.vi" Type="VI" URL="../NEO VI Example.llb/getdllversion.vi"/>
			<Item Name="GetErrorInfo.vi" Type="VI" URL="../NEO VI Example.llb/GetErrorInfo.vi"/>
			<Item Name="geterrormessages.vi" Type="VI" URL="../NEO VI Example.llb/geterrormessages.vi"/>
			<Item Name="getfiresettings.vi" Type="VI" URL="../NEO VI Example.llb/getfiresettings.vi"/>
			<Item Name="getmessages.vi" Type="VI" URL="../NEO VI Example.llb/getmessages.vi"/>
			<Item Name="GetTimeStampForMsg.vi" Type="VI" URL="../NEO VI Example.llb/GetTimeStampForMsg.vi"/>
			<Item Name="getvcan3setting.vi" Type="VI" URL="../NEO VI Example.llb/getvcan3setting.vi"/>
			<Item Name="icsneo40.dll" Type="Document" URL="icsneo40.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="icsneo40.dll" Type="Document" URL="../../../../../Windows/System32/icsneo40.dll"/>
			<Item Name="openneodevice.vi" Type="VI" URL="../NEO VI Example.llb/openneodevice.vi"/>
			<Item Name="parse message read.vi" Type="VI" URL="../NEO VI Example.llb/parse message read.vi"/>
			<Item Name="sendconfiguration.vi" Type="VI" URL="../NEO VI Example.llb/sendconfiguration.vi"/>
			<Item Name="setbitrate.vi" Type="VI" URL="../NEO VI Example.llb/setbitrate.vi"/>
			<Item Name="setfiresettings.vi" Type="VI" URL="../NEO VI Example.llb/setfiresettings.vi"/>
			<Item Name="setvcan3setting.vi" Type="VI" URL="../NEO VI Example.llb/setvcan3setting.vi"/>
			<Item Name="transmit messages.vi" Type="VI" URL="../NEO VI Example.llb/transmit messages.vi"/>
			<Item Name="txmessages raw.vi" Type="VI" URL="../NEO VI Example.llb/txmessages raw.vi"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
