import { memo } from 'react';
import { Navigate, Route, Routes } from 'react-router';

import Dashboard from 'app/main/Dashboard';
import Devices from 'app/main/Devices';
import Help from 'app/main/Help';
import HardwareStatus from 'app/status/HardwareStatus';
import MqttStatus from 'app/status/MqttStatus';
import NetworkStatus from 'app/status/NetworkStatus';
import Status from 'app/status/Status';
import SystemLog from 'app/status/SystemLog';
import Version from 'app/status/Version';
import { Layout } from 'components';

const AuthenticatedRouting = memo(() => {
  return (
    <Layout>
      <Routes>
        <Route path="/dashboard/*" element={<Dashboard />} />
        <Route path="/devices/*" element={<Devices />} />
        <Route path="/help/*" element={<Help />} />

        <Route path="/status/*" element={<Status />} />
        <Route path="/status/hardwarestatus/*" element={<HardwareStatus />} />
        <Route path="/status/log" element={<SystemLog />} />
        <Route path="/status/mqtt" element={<MqttStatus />} />
        <Route path="/status/network" element={<NetworkStatus />} />
        <Route path="/status/version" element={<Version />} />

        <Route path="/*" element={<Navigate to="/dashboard" />} />
      </Routes>
    </Layout>
  );
});

export default AuthenticatedRouting;
