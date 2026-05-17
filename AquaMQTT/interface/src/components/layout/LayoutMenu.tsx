import { memo } from 'react';

import AssessmentIcon from '@mui/icons-material/Assessment';
import CategoryIcon from '@mui/icons-material/Category';
import LiveHelpIcon from '@mui/icons-material/LiveHelp';
import StarIcon from '@mui/icons-material/Star';
import { Divider, List } from '@mui/material';

import LayoutMenuItem from 'components/layout/LayoutMenuItem';
import { useI18nContext } from 'i18n/i18n-react';

const LayoutMenuComponent = () => {
  const { LL } = useI18nContext();

  return (
    <>
      <List component="nav">
        <LayoutMenuItem icon={StarIcon} label="Dashboard" to={`/dashboard`} />
        <LayoutMenuItem icon={CategoryIcon} label={LL.DEVICES()} to={`/devices`} />
        <Divider />
      </List>
      <List style={{ marginTop: `auto` }}>
        <LayoutMenuItem
          icon={AssessmentIcon}
          label={LL.STATUS_OF('')}
          to="/status"
        />
        <LayoutMenuItem icon={LiveHelpIcon} label={LL.HELP()} to={`/help`} />
      </List>
    </>
  );
};

const LayoutMenu = memo(LayoutMenuComponent);

export default LayoutMenu;
