import { useMemo } from 'react';
import type { FC } from 'react';

import type { RequiredChildrenProps } from 'utils';

import { AuthenticatedContext, AuthenticationContext } from './context';

const Authentication: FC<RequiredChildrenProps> = ({ children }) => {
  const me = { username: 'admin', admin: true };

  const obj = useMemo(
    () => ({
      signIn: () => {},
      signOut: () => {},
      refresh: async () => {},
      me
    }),
    []
  );

  return (
    <AuthenticationContext.Provider value={obj}>
      <AuthenticatedContext.Provider value={obj}>
        {children}
      </AuthenticatedContext.Provider>
    </AuthenticationContext.Provider>
  );
};

export default Authentication;
