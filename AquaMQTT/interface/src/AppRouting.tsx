import { memo } from 'react';
import type { FC } from 'react';
import { Navigate, Route, Routes } from 'react-router';

import AuthenticatedRouting from 'AuthenticatedRouting';
import { Authentication } from 'contexts/authentication';

const AppRouting: FC = memo(() => {
  return (
    <Authentication>
      <Routes>
        <Route path="/" element={<Navigate to="/dashboard" replace />} />
        <Route path="/*" element={<AuthenticatedRouting />} />
      </Routes>
    </Authentication>
  );
});

export default AppRouting;
