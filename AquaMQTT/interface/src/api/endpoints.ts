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
  const data = response.data;
  if (typeof data === 'string') {
    try {
      return JSON.parse(data);
    } catch {
      return data;
    }
  }
  return data as unknown;
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
