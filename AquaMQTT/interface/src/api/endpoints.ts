import { type AlovaXHRResponse, xhrRequestAdapter } from '@alova/adapter-xhr';
import { createAlova } from 'alova';
import ReactHook from 'alova/react';

export const ACCESS_TOKEN = 'access_token' as const;

export const clearTokenCache = (): void => {};

const handleResponse = async (response: AlovaXHRResponse) => {
  if (response.status === 205) {
    throw new Error('Reboot required');
  }
  if (response.status === 400) {
    throw new Error('Request Failed');
  }
  if (response.status >= 400) {
    throw new Error(response.statusText);
  }
  return (await response.data) as unknown;
};

export const alovaInstance = createAlova({
  statesHook: ReactHook,
  cacheFor: null,
  requestAdapter: xhrRequestAdapter(),
  responded: {
    onSuccess: handleResponse
  }
});

export const alovaInstanceGH = createAlova({
  baseURL: 'https://api.github.com/repos/Psychotonikum/AquaMQTT/releases',
  statesHook: ReactHook,
  requestAdapter: xhrRequestAdapter()
});
