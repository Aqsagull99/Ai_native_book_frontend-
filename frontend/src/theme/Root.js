import React from 'react';
import ProvidersWrapper from '../ProvidersWrapper';

export default function Root({children}) {
  return <ProvidersWrapper>{children}</ProvidersWrapper>;
}